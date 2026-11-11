# 확산 정책 헤드(ENABLE_POLICY_DIFFUSION) 파인튜닝 가능성 — CPU 시험 (11.11, 집 PC) · **GPU 미검증**

## 왜

(가) 다봉 평균(중앙 장애물을 Nav2 가 좌우 반반으로 피함, `side_vs_geometry_11_11`)은 MSE 회귀 헤드의 전형적인 약점이다. X-Mobility 에는 모드를 고르는 확산 정책 헤드가 있고(`configs/base_train_config.gin` `ENABLE_POLICY_DIFFUSION`, 기본 False) 평가 · 노드도 확산 분기가 있어 학습 손잡이로 (가) 를 풀 수 있는지 봤다.

## 한 일 (CPU, 학습 2 스텝)

- 사전학습 ckpt(339c68f9e027)에서 `--gin ENABLE_POLICY_DIFFUSION=True --allow-missing`: 넣은 키 281, missing 148 = 모두 `model.action_policy.policy_diffuser.*`(새 헤드), unexpected = semantic_decoder 61(정상) + `policy_mlp.*` 16(버리는 MLP 헤드).
- 손실(fp32, B=2 × S=4): 스텝 0 action 31.63 · path 5.91 · kl 0.384, 스텝 1 29.04 · 5.13 · 0.282 — 유한. 이때 action = 30 × 노이즈 MSE 라 `validation/losses/action`(ckpt 고르는 기준)은 명령 정확도가 아님 → ckpt 는 eval_offline 으로 골라야 함.
- 새 헤드(ConditionalUnet1D) 55.1 M 파라미터(MLP 21.0 M), 학습 파라미터 86.1 → 120.2 M. 추론 5 스텝(action_policy.py 고정) CPU B=1 헤드 58 ms(MLP 2 ms).

## 막는 것 — X-Mobility 5837aea 버그 3 · 우리 쪽 1

1. **샘플러(결정적)** — `diffusion_policy.py` `denoising()`: 초기 노이즈에 `init_noise_sigma` 를 안 곱하고, 학습 때와 달리 `scale_model_input` 한 값을 모델에 넣고, `step()` 에 스케일된 표본을 넘김. 노이즈를 정확히 맞히는 가짜 모델에 ±ω 두 모드를 줘도 50 스텝이면 4000 표본 **모두 평균(ω≈0)**, 5 스텝이면 57 % 가 0 근처 — **고치지 않으면 확산 헤드도 직진**. 루프를 일관되게 고치면 5 · 50 스텝 모두 두 모드로 50 : 49(오차 0.8 %).
2. **정규화** — `encode_policy()` 가 차원을 잘못 잘라 action 6칸을 모두 ÷4(속도 ÷2 · 각속도 ÷2 가 겹침), `decode_policy()` 는 ×2 만 → 출력이 참값의 절반(v 0.8 → 0.4, ω 0.5 → 0.25).
3. **in-place** — `encode_policy()` 가 `batch['action']` 을 그 자리에서 바꿈 → fp32 역전파에서 "modified by an inplace operation" 오류(AMP 에서는 안 남, CUDA fp16 미검증). `.clone()` 으로 해결.
4. **우리 쪽** — `finetune.py --prev-action-source exec` 는 forward 전체의 `batch['action']` 을 실행 명령으로 바꿈 → 확산 헤드가 교사가 아닌 실행 명령(41 % 가 다름)을 배움. 확산 헤드와 쓰려면 RSSM 입력만 바꾸게 고치거나 `teacher`.

그 밖: diffusers `LMSDiscreteScheduler` 가 scipy 를 씀 — `.venv-xm` 과 노드 환경에 scipy 설치 필요(Kaggle 은 있음). CPU 에서는 16-mixed 가 bf16 로 바뀌어 역전파 실패 → `--gin 'finetune.precision="32-true"'`(GPU 는 16-mixed 그대로).

## 비용 (추정, 미검증)

- 학습 forward 마다 지표용 50 스텝 디노이즈가 돌아 스텝 시간이 약 2배 → 5060 Ti 에서 에폭 85–100 분, 10 에폭 14–16 h(14 h 상한 넘김). 학습 때 디노이즈를 빼면 +5–10 %.
- GPU 메모리 +0.7 GB 안팎. 노드 추론 +10–25 ms/프레임(5 Hz 200 ms 안).
- 헤드를 처음부터 학습(55 M, lr 1e-5 OneCycle) — 에폭이 더 필요하거나 헤드 lr 을 따로 높여야 할 수 있음. 고정 추론 노이즈면 상태마다 모드가 결정적으로 정해짐.

## 뜻

- 원리상 (가) 의 처방 후보지만, 쓰려면 위 1–3 을 우리 레포 쪽 패치(finetune · eval_offline · 노드 모두)로 고치고 GPU 에서 검증해야 함 — 11.16 기능 동결 전에는 빠듯.
- (가) 의 1순위 처방은 여전히 같은 쪽 교사 데이터(재녹화). 확산 헤드는 그다음 후보.
