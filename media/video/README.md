# 주행 영상

이 디렉터리는 본선 연속 주행과 팀 촬영 콘 구간의 공개용 하이라이트를 보존한다. MP4는 브라우저 재생 호환성을 위해 H.264/yuv420p로 변환하고 음성·원본 메타데이터를 제거했다.

| 파일 | 장면 | 원본 구간 | 규격 | 크기 | SHA-256 |
|---|---|---:|---|---:|---|
| `competition_drive_2x.mp4` | 본선 차선·장애물 연속 주행 | 공식 방송 4:56:45–4:57:11 | 13.0 s, 1280×720, 30 fps, 2× | 4,649,790 B | `04b29b02ee843b70f810299b51be3b48537da01daa9af54fe875268de394f3c5` |
| `competition_drive_2x.gif` | README 자동재생 preview | 위 MP4와 동일 | 13.0 s, 640×360, 12 fps | 11,748,731 B | `363375269145a663e530024b9eb6df40ff3d01d38caa92a7e9fc3b57c28ff5c0` |
| `cone_course_run.mp4` | 실차 콘 구간 | 2–18 s | 16.0 s, 960×540, 24 fps | 2,427,902 B | `3f2572c9d804f66ddbb24c79a30488a42ae7ccd065eab21d772745f925a74866` |

`competition_drive_2x.mp4`는 [국민대학교 공식 방송](https://youtu.be/CcfXS3UFL0A?t=17805)에서 차량이 멈추거나 사람에게 가리지 않고 계속 주행하는 26초를 선택해 2배속 13초로 만들었다. 사용자는 해당 편집 영상을 저장소에 포함할 권한이 있다고 확인했으며 원 방송의 표기와 출처를 유지했다.

`cone_course_run.mp4`는 팀 소유 원본 `20260807_kakaotalk_run_01.mp4`의 2–18초 구간을 사용했다. 두 영상은 주행 장면을 보여 주는 정성 자료이며 궤적 오차나 검출 정확도를 측정한 결과는 아니다.

재생 규격은 다음 명령으로 확인할 수 있다.

```bash
for video in media/video/*.mp4; do
  ffprobe -v error \
    -show_entries format=duration,size:stream=codec_name,width,height,r_frame_rate \
    "$video"
done
```
