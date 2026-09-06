# 주행 영상

`cone_course_run.mp4`는 2026년 국민대 대회 준비 중 촬영한 실차 콘 구간 테스트의 공개용 하이라이트입니다.

| 항목 | 값 |
|---|---|
| 길이 | 16.0 s |
| 영상 | H.264, 960×540, 24 fps, yuv420p |
| 음성 | 제거 |
| 파일 크기 | 2,427,902 bytes |
| SHA-256 | `3f2572c9d804f66ddbb24c79a30488a42ae7ccd065eab21d772745f925a74866` |

원본 `20260807_kakaotalk_run_01.mp4`의 2–18초 구간을 사용했습니다. 브라우저 재생 호환성과 저장소 용량을 위해 HEVC 원본을 H.264로 변환했고, 음성과 컨테이너 메타데이터는 제거했습니다. 영상은 주행 장면을 보여 주는 정성 자료이며 궤적 오차나 검출 정확도를 측정한 결과는 아닙니다.

재생 규격은 다음 명령으로 확인할 수 있습니다.

```bash
ffprobe -v error \
  -show_entries format=duration,size:stream=codec_name,width,height,r_frame_rate \
  media/video/cone_course_run.mp4
```
