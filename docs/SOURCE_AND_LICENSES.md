# 소스와 라이선스

## 팀 구현

다음 디렉터리는 국민대 대회용으로 통합·개발한 코드입니다.

- `ros2_ws/src/cam`
- `ros2_ws/src/custom_interfaces`
- `ros2_ws/src/mission_cone_drive`
- `ros2_ws/src/xycar_motor_native`
- `simulation/xycar_gz_sim`
- `tools`

패키지별 `package.xml`에 라이선스가 표시된 경우 해당 선언을 따릅니다. `cam`과 `custom_interfaces`에는 아직 유효한 라이선스 선언이 없으므로, 저작권자의 별도 허가 없이 재사용할 수 있다는 의미로 해석하면 안 됩니다. 이 저장소 전체에 적용되는 top-level 오픈소스 라이선스는 두지 않았습니다.

## 외부 드라이버

외부 코드는 원래의 라이선스 파일과 package metadata를 유지한 채 `ros2_ws/src/vendor` 아래에 분리했습니다.

| 구성요소 | 출처 | 선언된 라이선스 |
|---|---|---|
| `usb_cam` | [ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam) | BSD |
| `vesc_ros2` | [f1tenth/vesc](https://github.com/f1tenth/vesc) | BSD |
| `xycar_lidar` | [YDLIDAR/xycar_lidar](https://github.com/YDLIDAR/xycar_lidar) | MIT |
| bundled YDLidar SDK | [YDLIDAR/YDLidar-SDK](https://github.com/YDLIDAR/YDLidar-SDK) | bundled `LICENSE.txt` 참조 |
| `xycar_cam` | package metadata | Apache-2.0 |
| `xycar_imu` | [klintan/xycar_imu](https://github.com/klintan/xycar_imu) | bundled `LICENSE` 참조 |

각 구성요소를 배포하거나 수정할 때는 해당 디렉터리의 `LICENSE`와 notices를 확인해야 합니다.

## 대회 제공 자료와 기록물

규정 PDF와 DXF 원본은 저장소에 포함하지 않았습니다. Gazebo world는 로컬 코스 도면을 분석해 생성한 개발 산출물이며 원본 도면을 대체하거나 재배포하는 파일로 제공하지 않습니다.

`media/hardware`, `media/calibration`, `media/competition`의 사진은 사용자가 제공한 팀 촬영 자료를 문서용으로 crop/resize하고 EXIF를 제거한 것입니다. `media/video/cone_course_run.mp4`는 사용자가 제공한 팀 소유 원본에서 잘라 H.264로 변환하고 음성을 제거한 하이라이트이며, 구간과 checksum은 [`media/video/README.md`](../media/video/README.md)에 기록했습니다. 원본 전체 영상, rosbag, 학습 데이터셋과 참가자 개인정보는 포함하지 않습니다.

대회 공식 방송은 저장소에 재업로드하지 않습니다. README의 thumbnail은 사용자가 제공한 화면 캡처이고, 클릭하면 원 게시물의 timestamp로 이동합니다. 공식 방송 자체의 권리는 원 게시자에게 있습니다.

