# 파트 간 인계 규약

간트의 병목은 전부 **파트 사이 인계 지점**입니다. 각 문서가 넘기는 쪽과 받는 쪽의 계약입니다.

```
A sensor ──a2b──▶ B slam ──b2c──▶ C recon ──c2d──▶ D sim ──d2e──▶ E e2e
                                                    ▲                 │
                                                    └──────e2d────────┘
```

| 문서 | 방향 | 상태 |
| --- | --- | --- |
| [a2b_rosbag.md](a2b_rosbag.md) | A → B | 초안 |
| [b2c_map.md](b2c_map.md) | B → C | 초안 |
| [c2d_usd.md](c2d_usd.md) | C → D | 초안 |
| [d2e_teacher.md](d2e_teacher.md) | D → E | 초안 |
| [e2d_model.md](e2d_model.md) | E → D | 초안 |
| [topics.md](topics.md) | 공용 — 토픽 · frame_id · 좌표계 · 단위 · 파일 형식 | 초안 |

**토픽 이름 · frame_id · 좌표계 · 단위는 [`topics.md`](topics.md) 한 곳에 모읍니다.** 파트마다 따로 정하지 마세요.
