# Qualifying Round Evaluation

예선 결과 수치와 계산 근거를 별도로 보존했다. 원본 simulator replay나 timing log가 공개 저장소에 없으므로 측정 원시 데이터인 것처럼 표현하지 않고 사용자 제공 결과와 leaderboard 캡처를 구분했다.

## Result Summary

| Field | Value | Evidence |
|---|---:|---|
| Initial 3-lap time | 276.00 s | 사용자 제공 개발 기록 |
| Final best score | 151.32 s (2:31.32) | 사용자 제공 결과, `../media/leaderboard.png` |
| Displayed 3-lap total | 149.32 s (2:29.32) | leaderboard 캡처 표시값 |
| Rank | 9 | leaderboard 캡처 |
| Total participants | 132 teams | 사용자 제공 최종 정보 |
| Submissions | 11 | leaderboard 캡처 |
| Leaderboard last update | 2026-06-26 18:35:59 | leaderboard 캡처 |

이 포트폴리오의 대표 최종 기록은 사용자 요청과 leaderboard의 `Best Score` 열에 맞춰 151.32초로 통일했다. 같은 화면에 표시된 `3-Lap Total` 149.32초는 원본 화면을 충실히 보존하기 위해 별도 항목으로 남겼으며 대표 기록과 섞지 않았다.

## Improvement Calculation

```text
absolute reduction = 276.00 - 151.32
                   = 124.68 s

relative reduction = 124.68 / 276.00 × 100
                   = 45.173913... %
                   ≈ 45.17 %
```

초기 대비 최종 3-lap time은 124.68초 감소했고 상대 감소율은 45.17%였다. 이 개선은 PID 단독 성과가 아니라 lane perception, path generation, FSM transition, 좌회전 recovery, speed policy, lap/finish 판정까지 포함한 전체 pipeline 개선 결과로 해석했다.

## Machine-Readable Data

[`run_summary.csv`](run_summary.csv)에 문서가 사용하는 입력값과 evidence type을 기록했다. 원시 replay가 추가되면 동일한 디렉터리에 별도 파일을 넣고 측정값과 사용자 제공값을 분리할 수 있다.

## Limitations

- 132팀 전체 목록은 제공된 캡처에 모두 나타나지 않는다. 캡처 상단에는 50개 팀이 표시된다.
- 영상 파일 길이에는 시작 대기와 화면 촬영 맥락이 포함되므로 공식 주행 시간 측정에 사용하지 않았다.
- 원본 예선 source와 replay가 없어 steering error, lane detection accuracy, mission별 시간은 사후 정량화하지 않았다.
- `Best Score`와 `3-Lap Total`의 2초 차이는 화면에 보이는 그대로 기록했으며 근거 없이 산식이나 penalty 원인을 추정하지 않았다.
