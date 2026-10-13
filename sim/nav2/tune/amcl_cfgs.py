"""AMCL 설정 후보 — sim/nav2/nav2_params.yaml 의 amcl 만 바꾼 logs/nav2/amcl_tune/cfg_<이름>.yaml (10.14 2차 비교판).
기본 설정이 이미 10.14 고른 값(c_a002)이라 combo 덮어쓰기는 그 값과 같음 — 새 후보를 V 에 더해 쓰면 됨."""
import copy, yaml
base = yaml.safe_load(open("sim/nav2/nav2_params.yaml"))
combo = {"z_hit": 0.9, "z_rand": 0.1, "sigma_hit": 0.1, "max_beams": 120, "update_min_d": 0.1, "update_min_a": 0.1, "resample_interval": 1}
V = {"c_rep": {}, "c_a002": {"alpha1": 0.02, "alpha2": 0.02, "alpha3": 0.02, "alpha4": 0.02}, "c_b60": {"max_beams": 60},
     "c_b360": {"max_beams": 360}, "c_p5k": {"min_particles": 1000, "max_particles": 5000},
     "c_rec": {"recovery_alpha_slow": 0.001, "recovery_alpha_fast": 0.1}, "c_s02": {"sigma_hit": 0.2}, "c_vis": {}}
for k, ov in V.items():
    c = copy.deepcopy(base); c["amcl"]["ros__parameters"].update(combo); c["amcl"]["ros__parameters"].update(ov)
    yaml.safe_dump(c, open(f"logs/nav2/amcl_tune/cfg_{k}.yaml", "w"), allow_unicode=True, sort_keys=False)
print(" ".join(V))
