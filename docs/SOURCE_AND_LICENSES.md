# 소스와 라이선스

## 팀 구현

다음 디렉터리는 국민대 대회용으로 통합·개발한 코드이다.

- `ros2_ws/src/cam`
- `ros2_ws/src/custom_interfaces`
- `ros2_ws/src/mission_cone_drive`
- `ros2_ws/src/xycar_motor_native`
- `simulation/xycar_gz_sim`
- `tools`

패키지별 `package.xml`에 라이선스가 표시된 경우 해당 선언을 따른다. `cam`과 `custom_interfaces`에는 아직 유효한 라이선스 선언이 없으므로, 저작권자의 별도 허가 없이 재사용할 수 있다는 의미로 해석하면 안 된다. 이 저장소 전체에 적용되는 top-level 오픈소스 라이선스는 두지 않았다.

## 외부 드라이버

외부 코드는 원래의 라이선스 파일과 package metadata를 유지한 채 `ros2_ws/src/vendor` 아래에 분리했다.

| 구성요소 | 출처 | 선언된 라이선스 |
|---|---|---|
| `usb_cam` | [ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam) | BSD |
| `vesc_ros2` | [f1tenth/vesc](https://github.com/f1tenth/vesc) | BSD |
| `xycar_lidar` | [YDLIDAR/xycar_lidar](https://github.com/YDLIDAR/xycar_lidar) | MIT |
| bundled YDLidar SDK | [YDLIDAR/YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) | bundled `LICENSE.txt` 참조 |
| `xycar_cam` | package metadata | Apache-2.0 |
| `xycar_imu` | [klintan/xycar_imu](https://github.com/klintan/xycar_imu) | bundled `LICENSE` 참조 |

각 구성요소를 배포하거나 수정할 때는 해당 디렉터리의 `LICENSE`와 notices를 확인해야 한다.

## 대회 제공 자료와 기록물

규정 PDF와 DXF 원본은 저장소에 포함하지 않았다. Gazebo world는 로컬 코스 도면을 분석해 생성한 개발 산출물이며 원본 도면을 대체하거나 재배포하는 파일로 제공하지 않는다.

`media/hardware`, `media/calibration`, `media/competition`의 사진은 사용자가 제공한 팀 촬영 자료를 문서용으로 crop/resize하고 EXIF를 제거한 것이다. `media/video/cone_course_run.mp4`는 사용자가 제공한 팀 소유 원본에서 잘라 H.264로 변환하고 음성을 제거한 하이라이트이다. 원본 전체 영상, rosbag, 학습 데이터셋과 참가자 개인정보는 포함하지 않는다.

`media/video/competition_drive_2x.mp4`와 README용 GIF는 [국민대학교 공식 방송](https://youtu.be/CcfXS3UFL0A?t=17805)의 4:56:45–4:57:11 구간을 2배속·무음으로 편집한 자료이다. 사용자는 이 편집본을 저장소에 포함할 권한이 있다고 확인했다. 원 방송의 권리와 출처 표기는 원 게시자에게 있으며, 저장소는 선택 구간 외의 방송 전체를 포함하지 않는다. 모든 영상의 구간·규격·checksum은 [`media/video/README.md`](../media/video/README.md)에 기록했다.

`media/competition/final-ranking-7th.png`는 사용자가 제공한 본선 최종 결과 화면을 파일명만 정리해 보존한 자료다. 화면에서 건국대학교 Team SVE의 7위와 총 주행 시간 149.65초를 확인할 수 있다. 전체 참가 132팀과 본선 진출 22팀이라는 규모는 사용자 제공 정보로 구분했다.

`media/simulation/kookmin_gazebo_course.jpg`는 Kookmin course를 Gazebo에서 실행한 화면이며 simulation 개발 산출물이다.

## 예선 자료

`qualifying_round/media/qualifying_run.mp4`는 사용자가 제공한 팀 보유 원본 `KakaoTalk_20260625_060153117.mp4` 전체를 H.264/720p/BT.709로 변환하고 음성과 원본 메타데이터를 제거한 자료다. 공식 simulator 주행 과정을 보여 주지만 녹화 시작 대기까지 포함하므로 파일 길이를 공식 기록으로 사용하지 않았다.

`qualifying_round/media/leaderboard.png`는 사용자가 제공한 `KakaoTalk_20260626_183655459.png` 원본의 파일명만 정리한 자료다. Team SVE의 9위, best score 2분 31.32초, submissions 11이 표시되어 있다. 전체 참가 규모 132팀은 사용자 제공 최종 정보이며 캡처 한 장만으로 독립 검증할 수 없다는 범위를 예선 문서에 명시했다.

예선 원본 source와 simulator replay는 공개 Git history에 남아 있지 않다. 따라서 `qualifying_round/src/`를 만들거나 본선의 후속 adapter를 예선 원본처럼 재분류하지 않았다. 예선 상세 동작은 사용자 제공 개발 기록, 주행 영상과 leaderboard 캡처를 구분해 문서화했다.

