# 주행 영상

`cone_course_run.mp4`는 2026년 국민대 대회 준비 중 촬영한 실차 콘 구간 테스트의 공개용 하이라이트입니다. 브라우저 재생 호환성을 위해 H.264/yuv420p로 변환하고 음성·원본 메타데이터를 제거했습니다.

| 파일 | 장면 | 원본 구간 | 규격 | 크기 | SHA-256 |
|---|---|---:|---|---:|---|
| `cone_course_run.mp4` | 실차 콘 구간 | 2–18 s | 16.0 s, 960×540, 24 fps | 2,427,902 B | `3f2572c9d804f66ddbb24c79a30488a42ae7ccd065eab21d772745f925a74866` |

원본 `20260807_kakaotalk_run_01.mp4`의 2–18초 구간을 사용했습니다. 영상은 주행 장면을 보여 주는 정성 자료이며 궤적 오차나 검출 정확도를 측정한 결과는 아닙니다.

재생 규격은 다음 명령으로 확인할 수 있습니다.

```bash
for video in media/video/*.mp4; do
  ffprobe -v error \
    -show_entries format=duration,size:stream=codec_name,width,height,r_frame_rate \
    "$video"
done
```
