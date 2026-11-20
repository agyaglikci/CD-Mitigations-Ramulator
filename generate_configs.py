"""Generate Ramulator 2.0 YAML configs for ColumnDisturb and RowHammer/RowPress mitigations.

Usage examples:
    python3 generate_configs.py                                   # all configs, all single-core traces
    python3 generate_configs.py --configs salt ck_single --traces 429.mcf
    python3 generate_configs.py --mix benign attack --no-single   # 4-core mixes only

Config types (see CONFIGS):
    baseline, baseline_open            no mitigation (close-row cap 4 / open-row policy)
    salt                               SALT-C (subarray-level tracking, CDP neighbor propagation, PRAC ALERT/RFM)
    ck_single, ck_dual, ck_prob        MC-side ColumnKeeper-S / -D / -P (UnifiedDefense)
    ckd_indram, ckn_indram             in-DRAM ColumnKeeper-D (dual counters) / single-counter variant (PRAC ALERT/RFM)
    chronus_impress                    Chronus, RowPress via ImPress-P activation counting
    chronus_tmro, para, tmrocap        RowPress via a max-row-open cap (tmro) and a reduced threshold (ExPress)

ColumnDisturb configs sweep the ColumnDisturb activation threshold (max_act, THRESHOLDS);
RowHammer configs sweep the RowHammer threshold (rh_thresh).
"""
import math
import os
from pathlib import Path
import argparse


def _betacf(a, b, x):
    FPMIN = 1e-300; EPS = 3e-12; MAXIT = 300
    qab = a + b; qap = a + 1.0; qam = a - 1.0
    c = 1.0; d = 1.0 - qab * x / qap
    if abs(d) < FPMIN: d = FPMIN
    d = 1.0 / d; h = d
    for m in range(1, MAXIT + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        if abs(d) < FPMIN: d = FPMIN
        c = 1.0 + aa / c
        if abs(c) < FPMIN: c = FPMIN
        d = 1.0 / d; h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        if abs(d) < FPMIN: d = FPMIN
        c = 1.0 + aa / c
        if abs(c) < FPMIN: c = FPMIN
        d = 1.0 / d; de = d * c; h *= de
        if abs(de - 1.0) < EPS: break
    return h

def betainc(a, b, x):
    """Regularized incomplete beta I_x(a,b) — scipy.special.betainc drop-in."""
    if x <= 0.0: return 0.0
    if x >= 1.0: return 1.0
    lb = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
    bt = math.exp(lb + a * math.log(x) + b * math.log1p(-x))
    if x < (a + 1.0) / (a + b + 2.0):
        return bt * _betacf(a, b, x) / a
    return 1.0 - bt * _betacf(b, a, 1.0 - x) / b

def brentq(f, lo, hi, tol=1e-15, maxit=500):
    """Bisection root-finder — scipy.optimize.brentq drop-in (root is bracketed)."""
    flo = f(lo)
    for _ in range(maxit):
        mid = 0.5 * (lo + hi); fmid = f(mid)
        if fmid == 0.0 or (hi - lo) < tol:
            return mid
        if (flo < 0.0) != (fmid < 0.0):
            hi = mid
        else:
            lo, flo = mid, fmid
    return 0.5 * (lo + hi)


TCK_NS = 0.625
T_RFM_NS = 410

ROOT = Path(__file__).resolve().parent
TRACE_DIR = Path(os.environ.get("TRACE_DIR", str(ROOT / "cputraces")))

SUBARRAY_SIZE = 1024
ROWS_PER_BANK = 131072
NUM_SUBARRAYS = ROWS_PER_BANK // SUBARRAY_SIZE

def tck(ns):
    return math.ceil(ns / TCK_NS)

def mx(pair):
    return max(pair)

# Circuit-level (SPICE) worst-case timings of a 1024-row open-bitline subarray, (data 0, data 1) in ns.
# tRCD/tRAS/tRP -> nRCD 28, nRAS 60, nRP 29, nRC 89. tWR = cell (6.78) + non-cell (23.22) = 30 ns -> 48.
BASELINE_SPICE = {
    "tRCD": (16.90, 15.96), "tRAS": (20.70, 37.09),
    "tRP": (17.54, 17.76), "tWR": (30.0, 30.0),
}

def baseline_block():
    b = BASELINE_SPICE
    nRCD = tck(mx(b["tRCD"]))
    nRAS = tck(mx(b["tRAS"]))
    nRP = tck(mx(b["tRP"]))
    nWR = tck(mx(b["tWR"]))
    nRC = nRAS + nRP
    return (f"      nRCD: {nRCD}\n"
            f"      nRAS: {nRAS}\n"
            f"      nRP: {nRP}\n"
            f"      nWR: {nWR}\n"
            f"      nRC: {nRC}\n")

def _baseline_trc_ns():
    nRC = tck(mx(BASELINE_SPICE["tRAS"])) + tck(mx(BASELINE_SPICE["tRP"]))
    return nRC * TCK_NS


# ── ColumnDisturb mitigation thresholds (max_act = ColumnDisturb activation threshold N_CD) ──

def _salt_params(max_act):
    tRC_baseline = mx(BASELINE_SPICE["tRAS"]) + mx(BASELINE_SPICE["tRP"])
    bundle = int(T_RFM_NS // tRC_baseline)
    B = math.ceil(SUBARRAY_SIZE / bundle)
    # -2S: reservation for ColumnDisturb induced by periodic refresh (like _ck_params (Nth-2S)/S)
    apm_salt = (max_act - 2 * SUBARRAY_SIZE - 25) // (B + 1)
    # CDP: a subarray is also hammered by its two neighbors' activations
    apm_cdp = apm_salt // 3
    if apm_cdp - bundle < 1:
        return None
    return {"ath": apm_cdp * 2, "apm": apm_cdp, "bundle": bundle}

def _ck_params(max_act):
    npr = (max_act - 2 * SUBARRAY_SIZE) // SUBARRAY_SIZE
    if npr < 1: return None
    return {"ck_threshold": npr}

CKD_INDRAM_BUNDLE = 7

def _ckd_indram_params(max_act):
    abo_act = int(180 // _baseline_trc_ns())
    npr = (max_act - abo_act - 2 * CKD_INDRAM_BUNDLE - 2 * SUBARRAY_SIZE) // SUBARRAY_SIZE
    if npr < 2: return None
    return {"ckd_threshold": npr * CKD_INDRAM_BUNDLE, "ckd_bundle": CKD_INDRAM_BUNDLE}

YEAR_MS = 365 * 24 * 3600 * 1000
TARGET_P_YEAR = 1e-3
TREFW_MS = 32

def _one_minus_pow(p, n):
    return -math.expm1(math.log1p(-p) / n)

def _refresh_probability(Nth, S, tRC_ns, margin_mult, tf_mult):
    effective = Nth - margin_mult * S
    if effective - S < 1:
        return None
    t_f_ns = (1 + tf_mult * S) * tRC_ns
    M = (TREFW_MS * 1e6) // t_f_ns
    P_CD = _one_minus_pow(TARGET_P_YEAR, YEAR_MS // TREFW_MS)
    target_p_s = _one_minus_pow(P_CD, M + 1)
    f = lambda p: betainc(effective - S, S, 1 - p) - target_p_s
    if f(1e-12) * f(1 - 1e-12) > 0:
        return None
    return brentq(f, 1e-12, 1 - 1e-12)

def _ck_p_params(Nth):
    tRC = mx(BASELINE_SPICE["tRAS"]) + mx(BASELINE_SPICE["tRP"])
    p = _refresh_probability(Nth, SUBARRAY_SIZE, tRC, margin_mult=2, tf_mult=3)
    if p is None: return None
    return {"ck_p": p}


# ── RowHammer / RowPress thresholds (rh_thresh = RowHammer threshold N_RH) ──

# PARA (Kim et al. ISCA'14). F_target is back-computed from RowPress's PARA baseline
# (probability_threshold=0.034, T_RH=1000) and kept fixed; p is solved per rh_thresh.
PARA_F_TARGET = (1 - 0.034 / 2) ** 1000

def _para_params(rh_thresh):
    p = 2 * (1 - PARA_F_TARGET ** (1 / rh_thresh))
    if p >= 1.0:   # not achievable at this threshold (para.cpp rejects p >= 1)
        return None
    return {"threshold": p}

# RowPress Table 3 (§7.4), Mfr S 8Gb B-Die: tmro (ns) -> measured T'_RH for T_RH = 1000.
ROWPRESS_TMRO_TPRIME = {36: 1000, 66: 809, 96: 724, 186: 619, 336: 555, 636: 419}

def _para_tmro_params(rh_thresh, tmro_ns):
    t_prime = rh_thresh * ROWPRESS_TMRO_TPRIME[tmro_ns] / 1000.0
    return _para_params(t_prime)

def _chronus_nbo(n_rh, trc_ns):
    # Chronus §8: secure iff N_BO < N_RH - (activations possible in the 180 ns ABO window)
    nbo = n_rh - int(180 // trc_ns) - 1
    return nbo if nbo >= 1 else None

def _chronus_tmro_threshold(rh_thresh, tmro_ns):
    # Rounded down (more conservative). tmro=36 gives T' = rh.
    return max(1, int(rh_thresh * ROWPRESS_TMRO_TPRIME[tmro_ns] / 1000.0))

# Chronus RH-off sentinel: no row counter can reach it, so Chronus/PRAC never raise an ABO on their own.
# Used when Chronus/PRAC only provides the ALERT/RFM path for a ColumnDisturb mitigation.
CHRONUS_RH_OFF = 2000000000


NUM_INSTS = int(os.environ.get("NUM_INSTS", 250000000))
WARMUP_INSTS = int(os.environ.get("WARMUP_INSTS", 50000000))

# RandomTranslation address space = full DRAM capacity:
# 8 B/column x 2 ranks x 8 BG x 4 banks x 131072 rows x 1024 columns = 64 GB.
# Update this if the DRAM organization (preset/rank/channel_width) changes.
MAX_ADDR = 8 * 2 * 8 * 4 * (1 << 17) * (1 << 10)

def _frontend_yaml(trace_list, num_insts, warmup_insts):
    traces_yaml = "\n".join(f"    - {TRACE_DIR}/{t}" for t in trace_list)
    # A core whose trace name contains "atk" is an attacker: excluded from the end-of-run
    # condition and given a deterministic page pool pinned to a subarray span.
    attacker_cores = [i for i, t in enumerate(trace_list) if "atk" in t.lower()]
    attacker_yaml = f"  attacker_cores: {attacker_cores}\n" if attacker_cores else ""
    translation_attacker_yaml = f"    attacker_cores: {attacker_cores}\n" if attacker_cores else ""
    return f"""Frontend:
  impl: SimpleO3
  clock_ratio: 8
  num_expected_insts: {num_insts}
  num_warmup_insts: {warmup_insts}
  llc_capacity_per_core: 2MB
{attacker_yaml}  traces:
{traces_yaml}

  Translation:
    impl: RandomTranslation
    max_addr: {MAX_ADDR}
    subarray_rows: {SUBARRAY_SIZE}
    num_subarrays: {NUM_SUBARRAYS}
{translation_attacker_yaml}"""

COMMON_DRAM_ORG = """MemorySystem:
  impl: GenericDRAM
  clock_ratio: 3

  DRAM:
    impl: DDR5_VRR_RFM
    org:
      preset: DDR5_32Gb_x8
      channel_width: 64
      rank: 2
    timing:
      preset: DDR5_3200AN
"""

COMMON_DRAM_SUFFIX = """    RFM:
      BRC: 2
    RH_radius: 2
    drampower_enable: true
    voltage:
      preset: Default
    current:
      preset: Default
"""

CONTROLLER_BASE = """  Controller:
    impl: ReadDisturb
    debug: false
    wr_high_watermark: 0.8
    watchdog_max_wait_cycles: 200000000
"""

CONTROLLER_COMMON = """    Scheduler:
      impl: FRFCFS_PriorHit
    RefreshManager:
      impl: AllBank
    RowPolicy:
      impl: CloseRowPolicy
      cap: 4
"""

COLUMNKEEPER_TEMPLATE = """      - ControllerPlugin:
          impl: UnifiedDefense
          debug: false
          mitigation: {mode}
          alert_threshold: {ck_threshold}
          subarray_size: 1024
"""

COLUMNKEEPER_P_TEMPLATE = """      - ControllerPlugin:
          impl: UnifiedDefense
          debug: false
          mitigation: PROBABILISTIC
          alert_threshold: {ck_p}
          subarray_size: 1024
"""

COLUMNKEEPER_INDRAM_TEMPLATE = """      - ControllerPlugin:
          impl: ColumnKeeperInDRAM
          debug: false
          alert_threshold: {ckd_threshold}
          bundle: {ckd_bundle}
          subarray_size: 1024
"""

SALTC_TEMPLATE = """      - ControllerPlugin:
          impl: SALTC
          debug: false
          ath: {ath}
          apm: {apm}
          bundle: {bundle}
          subarray_size: 1024
          neighbor_propagation: true
"""

CHRONUS_PLUGIN = """      - ControllerPlugin:
          impl: Chronus
          debug: false
          abo_threshold: {abo_threshold}
          abo_act_ns: 180
          trefi_vrr_rate: -1
"""

# ImPress-P: each open period adds EACT = max(1, (tON + tRP) / tRC) to the row's counter.
CHRONUS_IMPRESS_PLUGIN = CHRONUS_PLUGIN.replace(
    "          impl: Chronus\n", "          impl: Chronus\n          impress: true\n")

PRAC_PLUGIN = """      - ControllerPlugin:
          impl: PRAC
          debug: false
          abo_threshold: {abo_threshold}
          abo_recovery_refs: {recovery_refs}
          abo_delay_acts: 4
          abo_act_ns: 180
"""
CKD_PRAC_RECOVERY_REFS = 4
SALT_PRAC_RECOVERY_REFS = 1

PARA_TEMPLATE = """      - ControllerPlugin:
          impl: PARA
          debug: false
          threshold: {threshold}
"""

ADDR_MAPPER = """
  AddrMapper:
    impl: MOP4CLXOR_RowSwap
"""

# Configs that use the open-row policy (no close-row cap); a row closes only on a conflict, REF, or tmro.
OPEN_ROW_CONFIGS = ("baseline_open", "chronus_impress", "chronus_tmro", "para", "tmrocap")
# Configs that force-close a row after tmro_ns (RowPress max-row-open cap).
TMRO_CONFIGS = ("chronus_tmro", "para", "tmrocap")

def generate_config(trace_list, config_type, max_act=16384, rh_thresh=1024, tmro_ns=636):
    yaml = (_frontend_yaml(trace_list, NUM_INSTS, WARMUP_INSTS)
            + COMMON_DRAM_ORG + baseline_block() + COMMON_DRAM_SUFFIX + CONTROLLER_BASE)
    if config_type in TMRO_CONFIGS:
        yaml += f"    max_open_ns: {tmro_ns}\n"
    if config_type in OPEN_ROW_CONFIGS:
        yaml += CONTROLLER_COMMON.replace("impl: CloseRowPolicy\n      cap: 4\n", "impl: OpenRowPolicy\n")
    else:
        yaml += CONTROLLER_COMMON

    if config_type == "salt":
        sp = _salt_params(max_act)
        if sp is None:
            return None
        yaml += "    plugins:\n"
        yaml += SALTC_TEMPLATE.format(**sp)
        yaml += PRAC_PLUGIN.format(abo_threshold=CHRONUS_RH_OFF, recovery_refs=SALT_PRAC_RECOVERY_REFS)
    elif config_type in ("ck_single", "ck_dual"):
        cp = _ck_params(max_act)
        if cp is None:
            return None
        yaml += "    plugins:\n"
        yaml += COLUMNKEEPER_TEMPLATE.format(mode="SINGLE" if config_type == "ck_single" else "DUAL", **cp)
        yaml += CHRONUS_PLUGIN.format(abo_threshold=CHRONUS_RH_OFF)
    elif config_type == "ck_prob":
        cp = _ck_p_params(max_act)
        if cp is None:
            return None
        yaml += "    plugins:\n"
        yaml += COLUMNKEEPER_P_TEMPLATE.format(**cp)
        yaml += CHRONUS_PLUGIN.format(abo_threshold=CHRONUS_RH_OFF)
    elif config_type in ("ckd_indram", "ckn_indram"):
        cp = _ckd_indram_params(max_act)
        if cp is None:
            return None
        yaml += "    plugins:\n"
        yaml += COLUMNKEEPER_INDRAM_TEMPLATE.format(**cp)
        if config_type == "ckn_indram":
            yaml += "          dual_counter: false\n"
        yaml += PRAC_PLUGIN.format(abo_threshold=CHRONUS_RH_OFF, recovery_refs=CKD_PRAC_RECOVERY_REFS)
    elif config_type == "chronus_impress":
        nbo = _chronus_nbo(rh_thresh, _baseline_trc_ns())
        if nbo is None:
            return None
        yaml += "    plugins:\n"
        yaml += CHRONUS_IMPRESS_PLUGIN.format(abo_threshold=nbo)
    elif config_type == "chronus_tmro":
        nbo = _chronus_nbo(_chronus_tmro_threshold(rh_thresh, tmro_ns), _baseline_trc_ns())
        if nbo is None:
            return None
        yaml += "    plugins:\n"
        yaml += CHRONUS_PLUGIN.format(abo_threshold=nbo)
    elif config_type == "para":
        pp = _para_tmro_params(rh_thresh, tmro_ns)
        if pp is None:
            return None
        yaml += "    plugins:\n"
        yaml += PARA_TEMPLATE.format(**pp)
    elif config_type not in ("baseline", "baseline_open", "tmrocap"):
        raise ValueError(f"unknown config type: {config_type}")

    yaml += ADDR_MAPPER
    return yaml


CD_CONFIGS = ["salt", "ck_single", "ck_dual", "ck_prob", "ckd_indram", "ckn_indram"]
RH_CONFIGS = ["chronus_impress", "chronus_tmro", "para", "tmrocap"]
CONFIGS = ["baseline", "baseline_open"] + CD_CONFIGS + RH_CONFIGS

# ColumnDisturb threshold (max_act) sweep.
THRESHOLDS = [128, 512, 2048, 8192, 32768, 131072, 524288]
SKIPPED_THRESHOLDS = [256, 1024, 4096, 16384, 65536, 262144]
# RowHammer threshold sweeps.
RH_THRESHOLDS = [128, 256, 512, 1024, 2048, 4096]
PARA_RH_THRESHOLDS = [64] + RH_THRESHOLDS
CHRONUS_RH_THRESHOLDS = [20, 32, 64] + RH_THRESHOLDS

TRACES_MEDIUM = [
    # SPEC CPU 2006
    "403.gcc", "429.mcf", "433.milc", "434.zeusmp", "435.gromacs", "436.cactusADM",
    "437.leslie3d", "444.namd", "445.gobmk", "447.dealII", "450.soplex", "456.hmmer",
    "458.sjeng", "459.GemsFDTD", "462.libquantum", "464.h264ref", "470.lbm", "471.omnetpp",
    "473.astar", "481.wrf", "482.sphinx3", "483.xalancbmk",
    # SPEC CPU 2017
    "500.perlbench", "502.gcc", "505.mcf", "507.cactuBSSN", "508.namd", "510.parest",
    "511.povray", "519.lbm", "520.omnetpp", "523.xalancbmk", "525.x264", "526.blender",
    "531.deepsjeng", "538.imagick", "541.leela", "544.nab", "549.fotonik3d", "557.xz",
]

# Multi-core mixes. Line format: Mix0,HHHH,trace1,trace2,trace3,trace4
# Workload name: "{kind}_{Mix}_{category}", e.g. benign_Mix0_HHHH, attack_Mix3_MMMA
MIX_FILES = {
    "benign": ROOT / "mixes" / "benign.mix",
    "attack": ROOT / "mixes" / "attack.mix",
}

def load_mixes(kinds, limit=None):
    mixes = {}
    for kind in kinds:
        path = MIX_FILES[kind]
        if not path.exists():
            print(f"[WARN] mix file not found: {path}")
            continue
        n = 0
        for line in path.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 3:
                continue
            name, category, trace_list = parts[0], parts[1], parts[2:]
            if limit is not None and n >= limit:
                break
            mixes[f"{kind}_{name}_{category}"] = trace_list
            n += 1
    missing = sorted({t for tl in mixes.values() for t in tl if not (TRACE_DIR / t).exists()})
    if missing:
        print(f"[WARN] mix traces not found in TRACE_DIR: {missing}")
    return mixes

def _mix_selected(mix_name, tokens):
    """mix_name = "benign_Mix3_MMMM". A token matches the full name or an exact part
    ("Mix3", "benign", "MMMM", "benign_Mix3"); exact match so "Mix5" does not catch Mix50."""
    kind, mix, category = mix_name.split("_", 2)
    keys = {mix_name, kind, mix, category, f"{kind}_{mix}", f"{mix}_{category}"}
    return any(t in keys for t in tokens)


# Single-core and multi-core configs go to separate directories.
CONFIG_DIR = ROOT / "auto_configs"
MIX_CONFIG_DIR = ROOT / "auto_configs_mix"

def generate_all(workloads, configs, thresholds=THRESHOLDS):
    """workloads: [(short_name, [trace, ...], is_mix), ...]"""
    n = 0
    skipped = 0

    def emit(path, make_yaml):
        """Never overwrite an existing non-empty config; only missing ones are generated."""
        nonlocal n, skipped
        if path.exists() and path.stat().st_size > 0:
            skipped += 1
            return
        yaml = make_yaml()
        if yaml is None:   # threshold not feasible for this config
            return
        path.write_text(yaml)
        n += 1

    for short, trace_list, is_mix in workloads:
        out_dir = MIX_CONFIG_DIR if is_mix else CONFIG_DIR
        out_dir.mkdir(exist_ok=True)
        for cfg in configs:
            if cfg in ("para", "chronus_tmro"):
                for rh in (PARA_RH_THRESHOLDS if cfg == "para" else CHRONUS_RH_THRESHOLDS):
                    for tmro in ROWPRESS_TMRO_TPRIME:
                        emit(out_dir / f"{short}_{cfg}_rh{rh}_tmro{tmro}.yaml",
                             lambda c=cfg, r=rh, t=tmro: generate_config(trace_list, c, rh_thresh=r, tmro_ns=t))
            elif cfg == "chronus_impress":
                for rh in CHRONUS_RH_THRESHOLDS:
                    emit(out_dir / f"{short}_{cfg}_rh{rh}.yaml",
                         lambda c=cfg, r=rh: generate_config(trace_list, c, rh_thresh=r))
            elif cfg == "tmrocap":
                for tmro in ROWPRESS_TMRO_TPRIME:
                    emit(out_dir / f"{short}_{cfg}_tmro{tmro}.yaml",
                         lambda c=cfg, t=tmro: generate_config(trace_list, c, tmro_ns=t))
            elif cfg in ("baseline", "baseline_open"):
                emit(out_dir / f"{short}_{cfg}.yaml", lambda c=cfg: generate_config(trace_list, c))
            else:   # ColumnDisturb mitigation: one config per max_act threshold
                for thr in thresholds:
                    emit(out_dir / f"{short}_{cfg}_t{thr}.yaml",
                         lambda c=cfg, t=thr: generate_config(trace_list, c, max_act=t))

    dirs = sorted({str(MIX_CONFIG_DIR if m else CONFIG_DIR) for _, _, m in workloads})
    print(f"Generated {n} configs in {', '.join(dirs)} ({skipped} already existed, skipped)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--configs", nargs="+", default=None,
                    help=f"config types to generate (exact names); default: all of {CONFIGS}")
    ap.add_argument("--traces", nargs="+", default=None,
                    help="single-core trace names; default: TRACES_MEDIUM")
    ap.add_argument("--mix", nargs="+", choices=["benign", "attack"], default=None,
                    help="also generate 4-core mixes from mixes/{benign,attack}.mix")
    ap.add_argument("--mix-limit", type=int, default=None, help="first N mixes of each mix file")
    ap.add_argument("--mix-names", nargs="+", default=None,
                    help="only mixes matching these tokens (e.g. Mix0 benign_Mix3 LLHH)")
    ap.add_argument("--no-single", action="store_true", help="skip single-core workloads")
    ap.add_argument("--skipped-thresholds", action="store_true",
                    help=f"use the in-between ColumnDisturb thresholds {SKIPPED_THRESHOLDS} instead of THRESHOLDS")
    args = ap.parse_args()

    if args.configs:
        unknown = [c for c in args.configs if c not in CONFIGS]
        if unknown:
            ap.error(f"unknown config types {unknown}; choose from {CONFIGS}")
    configs = args.configs or CONFIGS
    traces = args.traces or TRACES_MEDIUM

    workloads = [] if args.no_single else [(t.replace(".", "_"), [t], False) for t in traces]
    if args.mix:
        mixes = load_mixes(args.mix, limit=args.mix_limit)
        if args.mix_names:
            mixes = {k: v for k, v in mixes.items() if _mix_selected(k, args.mix_names)}
            if not mixes:
                print(f"[WARN] --mix-names {args.mix_names} matched no mix")
        workloads += [(k, v, True) for k, v in mixes.items()]
        print(f"  {len(mixes)} mixes ({', '.join(args.mix)}) x {len(configs)} configs")
    generate_all(workloads, configs,
                 thresholds=SKIPPED_THRESHOLDS if args.skipped_thresholds else THRESHOLDS)


if __name__ == "__main__":
    main()
