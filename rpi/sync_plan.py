"""RPi(jetcobot_126b)에서 실행되는 sync_plan — /joint_states를 실물 서보로 relay.

[2026-07-30] 이 파일의 정본(source of truth)은 이제 이 저장소의 `rpi/sync_plan.py`다.
그전까지는 RPi 안에서만 편집돼 버전 관리가 전혀 없었고(로컬에 사본이 없어 세션이 끝나면
내용을 잃었음), 실제로 그 때문에 직전 배포본을 재구성해야 하는 일이 있었다.

배포 경로(RPi):
  ~/smh_ws/src/mycobot_ros2_humble/mycobot_280/mycobot_280_moveit2_control/
    mycobot_280_moveit2_control/sync_plan.py

RPi의 `build/`가 `src/`를 심볼릭 링크하므로 scp 후 colcon 재빌드는 필요 없다.

⚠️ 배포 전에 RPi의 현재 파일을 먼저 가져와 이 파일과 diff할 것 — Pi에서 직접 편집된
내용이 있으면 그냥 scp하면 사라진다(과거에 실제로 Pi에서 speed 값을 직접 바꾼 적 있음).

⚠️ `sync_plan`은 같은 시리얼 포트(/dev/ttyJETCOBOT == /dev/ttyUSB0)를 쓰는 다른
프로세스(follow_display, 직접 pymycobot 스크립트)와 동시에 실행할 수 없다.

⚠️ 이 노드를 켜는 순간 실물은 로컬 시뮬레이션의 현재 관절값으로 즉시 움직인다 —
켜기 전에 로컬과 실물의 관절각이 일치하는지 반드시 확인할 것.

`src/mycobot_ros2/.../sync_plan.py`(vendored upstream 원본)와는 다른 파일이다.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
import os
import time
import math
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280
    # [2026-07-30] set_color를 논블로킹으로 보내기 위해 내부 프로토콜 상수를
    # 직접 씀 — 아래 _set_color_async 설명 참고.
    from pymycobot.common import ProtocolCode

# [2026-07-27] coord_to_goal_node.py의 GRIPPER_JOINT_NAME/GRIPPER_OPEN_POSITION/
# GRIPPER_CLOSED_POSITION과 반드시 동일하게 유지할 것 — 여기서 값이 갈리면
# open/close 판정 임계값이 어긋남. 이 워크스페이스가 처음 그리퍼 actuation을
# 시도한 세션이라, sync_plan은 그동안 팔 6축만 relay하고 그리퍼는 아예 몰랐음
# (gripper_controller가 rviz_order에 없어 무시됨, /joint_states에 값이 와도
# 실물로 전혀 안 나감) — 이 누락을 메움.
GRIPPER_JOINT_NAME = 'gripper_controller'
GRIPPER_OPEN_POSITION = 0.12
GRIPPER_CLOSED_POSITION = -0.6
# 두 값의 중간점 — 이보다 크면 "열기" 목표, 작으면 "닫기" 목표로 판정.
GRIPPER_MIDPOINT = (GRIPPER_OPEN_POSITION + GRIPPER_CLOSED_POSITION) / 2.0
GRIPPER_SPEED = 50
# 실물 그리퍼 타입: mycobot_280_m5_adaptive_gripper = adaptive gripper(1).
GRIPPER_TYPE = 1

# [2026-07-28, 그리퍼 단계 LED 표시] coord_to_goal_node.py의 GRASP_STEP_*
# 상수와 반드시 같은 매핑을 유지할 것(두 프로세스가 독립 실행 파일이라
# 값을 import로 공유하지 않음). 각 값은 (r, g, b), 0~255.
GRASP_STEP_COLORS = {
    0: (0, 0, 0),        # idle — 꺼짐
    1: (0, 0, 255),      # 1/5 정렬 — 파랑
    2: (0, 255, 255),    # 2/5 그리퍼 열기 — 시안
    3: (255, 255, 0),    # 3/5 직진 접근 — 노랑
    4: (255, 0, 0),      # 4/5 파지 — 빨강
    5: (0, 255, 0),      # 5/5 후퇴 — 초록
}

# [2026-07-30, 단계 점멸] 색상만으로는 구간을 가리기 어려운 문제가 있었음 —
# 후퇴(5/5)와 그 뒤 look pose 복귀가 둘 다 초록으로 표시돼서(복귀는 별도 단계
# 번호가 없고, coord_to_goal_node가 복귀 완료 후에야 IDLE을 발행함) 실물 관찰
# 시 "후퇴는 부드럽고 복귀는 떨린다"를 색으로 구분할 수 없었음.
# 그래서 파지(4)만 상시 점등으로 두고 나머지는 점멸시켜 구분 가능하게 함.
#
# 주기 선정: 사람 눈은 대략 10Hz를 넘으면 점멸을 연속광/미세 플리커로 인지하므로
# 20Hz는 상시 점등과 구별되지 않음. 0.25s 토글 = 2Hz 점멸이 "깜빡인다"로
# 확실히 보이는 표준적인 값.
BLINK_TOGGLE_PERIOD_SEC = 0.25
# 점멸하지 않고 상시 점등할 단계 — 파지(4)는 순간적으로 끝나는 단계라 점멸하면
# 오히려 놓치기 쉬움.
SOLID_STEPS = {4}

# [2026-07-29] 전체 궤적 재생(sync_trajectory 구독/재생 스레드)은 실물에서
# joint5가 목표 앞에서 반복적으로 흔들리다 멈추는 문제가 관측되어 제거하고
# 원래의 반응형 relay(listener_callback)로 복귀함.
#
# 대신 relay 쪽의 "구간 단위로 뚝뚝 끊김" 문제를 별도로 다룸: /joint_states는
# controller_manager update_rate=100Hz 그대로 발행되는데, 이 주기 그대로
# mc.send_angles()를 호출하면 mycobot280 펌웨어가 이전 가감속 프로파일이
# 끝나기도 전에 매번 새 목표로 재계산을 시작해 "정지-출발-정지"처럼 보임
# (시리얼 버퍼링 문제라기보다 펌웨어가 point-to-point 명령을 스트리밍용으로
# 설계되지 않았기 때문). 아래 두 상수로 전송 주기를 낮추고, 변화가 거의 없는
# 프레임은 재전송을 건너뜀.
THROTTLE_PERIOD_SEC = 0.05 #0.2 #0.05  # 최대 20Hz로 send_angles 호출을 제한
# [2026-07-30] 0.7 -> 0.1. 실측 원인 규명 결과:
# 0.7은 저속 구간의 명령 주기를 망가뜨리고 있었다. scaling 0.05(ω=4.5°/s)에서는
# THROTTLE_PERIOD_SEC(0.05s)당 관절이 0.225°만 움직이는데, 이게 0.7 미만이라
# 3.4프레임이 누적될 때까지 전송이 막혔다 — 실측 로그에서 접근 구간 dmax가
# 0.711~1.12°(=0.7 바로 위)에 뭉쳐 있고 dt가 0.169s(5.9Hz)로 늘어난 것이 증거.
# 순항 구간(scaling 0.2, dmax≈1.08°)은 17.2Hz로 정상이었으므로, 저속 구간만
# 초당 5.9회의 이산적 움직임 = 약 6Hz 덜컹거림이 됐다(사용자 실물 관측과 일치).
#
# 0.7이었던 이유는 _async 도입 전에는 send_angles 1회가 1.5초를 블로킹해서
# 명령 수를 줄이는 것이 이득이었기 때문 — 그 전제가 사라졌으므로 이제는 손해다.
#
# 값 선정 기준: 쓰는 가장 낮은 스케일(0.05 -> 프레임당 0.225°)에서 프레임이
# 안 막히도록 그보다 충분히 작아야 함. 0.1은 약 2.3배 여유.
# 이 상수의 원래 목적(정지 중 동일 명령 반복 전송 방지)은 그대로 유지된다 —
# FakeSystem 시뮬레이션은 센서 노이즈가 없어 정지 시 값이 완전히 동일하므로
# 0보다 큰 값이면 어떤 값이든 억제된다.
# 앞으로 이보다 더 낮은 스케일을 쓰려면 이 값도 함께 낮출 것.
ANGLE_EPSILON_DEG = 0.1     # 이 이하 변화는 노이즈로 보고 재전송 생략

# [2026-07-30, TASK A — 서보 speed 동적 계산] docs/SPEED_TUNING_HANDOFF.md 참고.
#
# send_angles(angles, speed)의 speed는 팔의 "속도"를 정하지 않는다 — 실제 속도는
# PC 쪽 MoveIt의 VELOCITY_SCALING이 /joint_states 타이밍에 이미 반영해서 보낸다.
# speed가 정하는 건 서보의 "추종 능력"뿐이다. 그래서 speed를 전역 상수로 두면,
# PC가 구간별로 다른 스케일을 쓸 때(예: 3/5 직진 접근은 접촉 구간이라 의도적
# 저속) 서보가 목표에 먼저 도착해 대기하다 다음 명령에 홱 움직이는 "가다 서다"가
# 된다. 그래서 speed를 매 명령의 실제 각속도에서 역산한다.
#
# SPEED_GAIN_K 유도: delta_max / elapsed는 그 자체로 관절 각속도 ω(°/s)다.
# joint_limits.yaml이 6축 전부 max_velocity=1.57 rad/s(≈90 °/s)이므로,
# "부드러움 실물 확인됨"으로 기록된 동작점(VELOCITY_SCALING=0.2 ↔ speed=40)에서
#   ω = 0.2 × 90 = 18 °/s,  K = 40 / 18 ≈ 2.22
# 교차 검증: speed = 2.22 × 90 × scaling = 200 × scaling → 핸드오프 문서가 실측
# 2점에서 얻은 경험식 `speed ≈ 200 × VELOCITY_SCALING`과 정확히 일치한다.
#
# 기준점 선택 이력: TASK A 배포 직전 Pi에는 speed=50이 하드코딩돼 있었으나(실물
# 튜닝 중 40에서 올린 값), 사용자 확인 결과 "부드러움이 확인된 기준"은 문서에
# 기록된 40이므로 40을 채택했다. 만약 50이 실제로 더 나은 것으로 판명되면
# K = 50/18 ≈ 2.78 (speed ≈ 250 × scaling)로 바꾸면 된다.
#
# ⚠️ 앵커 재검증 필요(2026-07-30): "speed=40이 부드러웠다"는 관측은 아래
# _async=True 수정 전, 즉 relay가 0.64Hz로 막혀 명령당 28°씩 나가던 시절에
# 얻은 것이다. 수정 후에는 명령당 0.9°로 정상화되므로 "부드러움"의 기준 자체가
# 달라졌다 — 최적 speed가 40이 아닐 수 있다. 다행히 이 공식은 speed = K × ω로
# 각속도 기반이라 dt가 1.5s→0.05s로 바뀌어도 ω(=18°/s @ scaling 0.2)는 그대로고
# speed=40이 그대로 나온다(dt에 불변). 그래도 실물 체감으로 K를 다시 확인할 것.
#
# 남은 불확실성: "서보 speed가 각속도에 선형 대응한다"는 가정은 미검증이다.
# 로그에 delta_max/elapsed/speed를 함께 찍으므로, 실물 로그의 평균 Δθ̄로
# K = 2.0 / Δθ̄ 를 역산해 이 2.22와 대조한 뒤 필요하면 교체할 것.
# [2026-07-30, A/B 토글] False로 두면 동적 계산을 끄고 FIXED_SPEED를 그대로
# 사용함 — 동적 계산(TASK A) 도입 전 동작과 같아짐. 실물 체감이 동적 도입 후
# 오히려 나빠졌다는 관측이 있어 A/B 비교가 가능하도록 남겨둠. 이 값만 바꾸고
# sync_plan을 재시작하면 되고, 다른 변경(_async, ANGLE_EPSILON_DEG)은 그대로
# 유지되므로 "동적 계산만" 분리해서 볼 수 있음.
USE_DYNAMIC_SPEED = True
# USE_DYNAMIC_SPEED=False일 때 쓰는 고정 speed. 동적 계산 도입 전 이 파일에
# 하드코딩돼 있던 값은 50이었고, 문서에 "부드러움 확인됨"으로 기록된 값은
# 40(scaling 0.2 기준)임. 40으로 두면 동적 계산이 순항 구간에서 내는 값과
# 같아져 비교가 깔끔함.
FIXED_SPEED = 40

# [2026-07-30, speed 노이즈 억제] 계산된 speed가 이 값 미만으로 바뀌면 직전에
# 보낸 speed를 그대로 재사용함. 이유: speed는 dmax/elapsed(유한차분)에서 나오는데
# dmax는 소수 3자리 반올림, elapsed는 0.05~0.06s로 흔들리므로, 등속 순항 중에도
# 결과가 ±1~2씩 노이즈를 탄다. 실측(전 구간 scaling 0.2)에서 정렬/복귀 구간의
# speed 변화율이 50%(초당 ~10회)였고, 그 83%는 순항(speed 40)이었음 — 즉 대부분이
# 진짜 속도 변화가 아니라 노이즈였다. send_angles에 speed가 바뀌어 들어오면 서보
# 펌웨어가 가감속 프로파일을 새로 계산하므로, 초당 10회 재계산이 미세한 떨림
# ("건들건들", 사용자 실물 관측)의 원인으로 판단됨.
# 4로 두면 순항 중 ±1~2 노이즈는 흡수하고, 구간이 실제로 바뀔 때(40 -> 20 -> 10)는
# 정상적으로 따라감. 램프 구간에서도 4단위로는 계단식으로 따라감.
SPEED_HYSTERESIS = 4

# [2026-07-30, 3차 — 2.22에서 1.7로 낮춤. 설계 전제 정정]
# 2.22는 "t ≈ T"(서보가 다음 명령이 올 때 목표에 도착)를 목표로 역산한 값이었고,
# 그게 핸드오프 문서의 "부드러움 조건"이었음. 그런데 그 조건 자체가 틀렸음 —
# t ≈ T는 "매 명령을 완주하고 멈춘다"는 뜻이다. 실측으로 확인:
#   명령당 0.902°, T=58ms, speed=39 -> 서보 속도 39/2.22=17.6°/s
#   -> 0.902° 이동에 t=51ms -> T보다 7ms 일찍 도착해서 대기
#   -> 초당 17번 도착-정지-재출발 = 미세 떨림("건들건들", 사용자 실물 관측)
# 이 떨림은 명령 주기(17.2Hz), 명령 단조성(방향 반전 0%), speed 안정성(이력으로
# 변화율 11.9%까지 낮춤), 명령 크기(86%가 8~14 인코더 스텝)를 모두 정상화한
# 뒤에도 남았으므로, 원인이 타이밍 조건 자체였다고 판단.
#
# 연속 운동에는 t > T가 필요하다 — 서보가 "아직 가는 중"일 때 새 목표가 와서
# fresh_mode=1이 진행 중 동작을 가로채면 멈춤 없이 이어진다.
# t = 1.15*T가 되도록 역산: speed를 0.88/1.15 = 0.765배 -> K = 2.22*0.765 ≈ 1.7
#
# 주의: speed를 올리는 것은 반대 방향으로 악화시킨다(더 일찍 도착 -> 대기 증가).
# 너무 낮추면 서보가 계획 궤적을 못 따라가 뒤처지는데, fresh_mode=1이 항상 최신
# 목표로 갈아타므로 뒤처짐은 누적되지 않고 명령 1개분(~0.9°) 수준에 머문다.
# 조정 지침: 여전히 떨리면 더 낮추고(1.5), 팔이 굼뜨거나 뒤처지면 올린다(1.9).
SPEED_GAIN_K = 1.7
# speed=0은 펌웨어에서 정의되지 않음 → 하한 필수.
SPEED_MIN = 5
SPEED_MAX = 100
# 첫 전송(직전 전송값이 없어 각속도를 낼 수 없음)에 쓰는 값. 낮게 잡아야 안전함
# — 첫 명령은 이동 거리가 얼마일지 모르는 상태라 느린 편이 낫다.
SPEED_FALLBACK = 20


class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_sync_plan")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription

        # self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        # self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        # if self.robot_m5:
        #     port = self.robot_m5
        # else:
        #     port = self.robot_wio
        # self.get_logger().info("port:%s, baud:%d" % (port, 115200))
        # self.mc = MyCobot280(port, 115200)
        # time.sleep(0.05)
        # if self.mc.get_fresh_mode() == 0:
        #     self.mc.set_fresh_mode(1)
        #     time.sleep(0.05)

        # 수정:
        port = "/dev/ttyJETCOBOT"
        self.get_logger().info("port:%s, baud:%d" % (port, 1000000))
        self.mc = MyCobot280(port, 1000000)
        time.sleep(0.1)

        # [fresh_mode — 결론 내리지 않음, 2026-07-30 현재 미해결]
        # pymycobot 공식 문서(MyCobot_280_en.md) 기준:
        #   0 = Interpolation — "Execute instructions sequentially in the form
        #       of a queue" (명령을 큐에 쌓아 순서대로 완결)
        #   1 = Refresh — "Always execute the latest command first" (진행 중인
        #       동작을 중단하고 최신 명령부터 다시 실행)
        #
        # 지금까지의 경과(어느 쪽도 결론이 아님):
        #   - upstream 원본 예제는 이 relay 용도로 1을 맞췄음(위 주석 처리된 코드)
        #   - 2026-07-29에 끊김 개선을 기대하고 0으로 바꿔 실험함
        #   - 이후 실물 튜닝 과정에서 다시 1로 돌려놓은 상태이며, 현재 코드도 1
        #   - 핸드오프 문서(docs/SPEED_TUNING_HANDOFF.md)에는 "1이 더 나빴다"는
        #     관측이 적혀 있으나, 그때는 speed가 하드코딩이었고 다른 변수를
        #     고정한 통제된 비교가 아니었으므로 그 기록만으로 0을 택할 근거는 안 됨
        #
        # 그래서 어느 모드가 낫다는 판단을 코드로 못박지 않고, 실물에서 실제로
        # 돌아가고 있던 값(1)을 그대로 유지한다. TASK A로 speed가 동적이 되면서
        # 명령당 도착 시간과 relay 주기의 관계 자체가 달라졌으므로, 이전 관측은
        # 그대로 적용되지 않는다 — 로컬(구간별 VELOCITY_SCALING)/리모트(speed,
        # THROTTLE_PERIOD_SEC) 변수를 하나씩 고정한 상태에서 0과 1을 다시 비교할
        # 것. 그 비교 없이 이 값을 뒤집지 말 것(양방향 모두).
        if self.mc.get_fresh_mode() != 1:
            self.mc.set_fresh_mode(1)
            time.sleep(0.1)
        self.get_logger().info(f'fresh_mode: {self.mc.get_fresh_mode()}')

        # RViz中目标顺序
        self.rviz_order = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]

        # [2026-07-27] 마지막으로 실물에 보낸 그리퍼 상태(0=open,1=close) —
        # /joint_states가 자주(수십 Hz) 들어오는데 매번 set_gripper_state를
        # 부르면 안 되니, 목표 상태가 바뀔 때만 전송.
        self._last_gripper_flag = None

        # [2026-07-28, release/refocus] True면 listener_callback이 실물로
        # 아무 명령도 안 보냄(release_servos()로 푼 토크가 다음 /joint_states
        # 메시지에서 바로 send_angles()로 재잠김되는 걸 막기 위한 플래그 —
        # 이게 없으면 release는 사실상 무의미함, /joint_states가 수십 Hz로
        # 들어오기 때문). refocus_servos()가 다시 False로 풀어줌.
        self._released = False

        # [2026-07-29] throttle 게이트용 타임스탬프 — 전송이 실제로 일어났는지와
        # 무관하게 매 통과 시점마다 갱신됨.
        self._last_send_time = self.get_clock().now()
        self._last_sent_angles = None
        # [2026-07-30, TASK A] "실제로 전송이 일어난" 시점만 기록하는 별도
        # 타임스탬프. _last_send_time을 재사용하면 안 된다 — 그건 ANGLE_EPSILON_DEG로
        # 전송이 생략된 프레임에서도 갱신되므로, 그걸로 경과 시간을 재면 저속
        # 구간(생략이 잦은 구간)에서 각속도가 과소평가되어 speed가 필요보다 낮게
        # 나온다. _last_sent_angles와 짝을 이루는 시간축이 이쪽이다.
        self._last_sent_time = None
        # [2026-07-30] 마지막으로 실물에 보낸 speed — SPEED_HYSTERESIS 판정용.
        self._last_sent_speed = None

        self.create_service(Trigger, 'release_servos', self._on_release_servos)
        self.create_service(Trigger, 'refocus_servos', self._on_refocus_servos)

        # [2026-07-28, 그리퍼 단계 LED 표시] coord_to_goal_node가 5단계 중
        # 현재 단계를 발행하면 그에 맞춰 상단 RGB LED 색을 바꿈(set_color).
        self.create_subscription(Int32, 'grasp_step', self._on_grasp_step, 10)

        # [2026-07-30, 단계 점멸] 위 BLINK_TOGGLE_PERIOD_SEC 설명 참고.
        self._grasp_step = 0
        self._blink_on = True
        self.create_timer(BLINK_TOGGLE_PERIOD_SEC, self._on_blink_timer)

    def _set_color_async(self, r, g, b):
        """LED 색을 논블로킹으로 전송.

        [2026-07-30] mc.set_color()를 그대로 쓰면 안 됨 — 그건 _mesg를
        _async 없이 호출하므로 send_angles와 똑같은 블로킹 경로를 탄다
        (응답 없는 명령에 wait_time 0.5s × 3회 재시도 = 호출당 1.5초, 위
        _async=True 주석 참고). 실제로 이 노드는 그동안 단계 전환마다 1.5초씩
        멈추고 있었고(사이클당 6회), 점멸을 넣으면 초당 여러 번 1.5초를 잡아
        팔 relay가 아예 죽는다.
        pymycobot의 공개 set_color()에는 _async를 넘길 구멍이 없어서 내부
        _mesg를 직접 호출함. LED는 응답을 확인할 필요가 없으므로 write-only로
        충분하다.
        """
        self.mc._mesg(ProtocolCode.SET_COLOR, r, g, b, _async=True)

    def _on_grasp_step(self, msg):
        color = GRASP_STEP_COLORS.get(msg.data)
        if color is None:
            self.get_logger().warn(f'알 수 없는 grasp_step 값: {msg.data}')
            return
        # 단계가 바뀌면 즉시 점등하고 점멸 위상을 리셋 — 전환을 놓치지 않게.
        self._grasp_step = msg.data
        self._blink_on = True
        r, g, b = color
        self._set_color_async(r, g, b)

    def _on_blink_timer(self):
        color = GRASP_STEP_COLORS.get(self._grasp_step)
        # idle(꺼짐)이거나 상시 점등 단계면 타이머가 건드리지 않음 —
        # 불필요한 시리얼 전송도 하지 않는다.
        if not color or color == (0, 0, 0) or self._grasp_step in SOLID_STEPS:
            return
        self._blink_on = not self._blink_on
        r, g, b = color if self._blink_on else (0, 0, 0)
        self._set_color_async(r, g, b)

    def _on_release_servos(self, request, response):
        self._released = True
        self.mc.release_all_servos()
        response.success = True
        response.message = (
            '서보 풀림 — 손으로 자유롭게 움직일 수 있음. relay 일시정지됨 '
            '(refocus_servos 호출 전까지 /joint_states 무시).'
        )
        self.get_logger().warn(response.message)
        return response

    def _on_refocus_servos(self, request, response):
        # 핵심: get_angles()로 지금 실제 물리 각도를 먼저 읽고 그 값 그대로
        # 재전송해야 제자리에서 안전하게 토크가 잠김(release 직전 마지막
        # 목표각으로 스냅되는 것 방지).
        current_angles = self.mc.get_angles()
        self.mc.send_angles(current_angles, 50)
        self._released = False
        # [2026-07-30, TASK A] 손으로 옮겨진 뒤라 직전 전송값 기준 각속도는
        # 의미가 없음 — 다음 전송이 SPEED_FALLBACK으로 시작하도록 이력을 비움.
        self._last_sent_angles = None
        self._last_sent_time = None
        self._last_sent_speed = None
        response.success = True
        response.message = (
            f'현재 위치 {current_angles}(도)에서 토크 재고정, relay 재개함. '
            '⚠️ 로컬 시뮬레이션(RViz)이 이 각도와 다르면 다음 /joint_states '
            '수신 즉시 그쪽으로 움직이니, 재개 전 로컬 시뮬레이션을 이 각도로 '
            '먼저 맞출 것.'
        )
        self.get_logger().warn(response.message)
        return response

    def _servo_speed_for(self, data_list, now):
        """[2026-07-30, TASK A] 이번에 보낼 목표까지의 실제 각속도에서 서보
        추종 speed를 역산함(위 SPEED_GAIN_K 설명 참고). 직전 전송 이력이 없으면
        SPEED_FALLBACK.

        반환값은 (speed, delta_max, elapsed) — 뒤 두 개는 SPEED_GAIN_K 재캘리
        브레이션용으로 로그에 남기기 위함.

        USE_DYNAMIC_SPEED=False면 FIXED_SPEED를 그대로 반환(동적 계산 도입 전
        동작). 단 delta_max/elapsed는 계속 계산해서 로그에 남김 — 고정/동적
        비교 시 같은 지표로 봐야 하므로.
        """
        if self._last_sent_angles is None or self._last_sent_time is None:
            return (FIXED_SPEED if not USE_DYNAMIC_SPEED else SPEED_FALLBACK), None, None

        elapsed = (now - self._last_sent_time).nanoseconds * 1e-9
        if elapsed <= 0.0:
            return (FIXED_SPEED if not USE_DYNAMIC_SPEED else SPEED_FALLBACK), None, elapsed

        if not USE_DYNAMIC_SPEED:
            delta_max = max(
                abs(a - b) for a, b in zip(data_list, self._last_sent_angles)
            )
            return FIXED_SPEED, delta_max, elapsed

        delta_max = max(
            abs(a - b) for a, b in zip(data_list, self._last_sent_angles)
        )
        # delta_max / elapsed = 관절 각속도(°/s).
        speed = int(round(SPEED_GAIN_K * delta_max / elapsed))
        speed = max(SPEED_MIN, min(SPEED_MAX, speed))

        # [2026-07-30] 이력(hysteresis) — 위 SPEED_HYSTERESIS 설명 참고.
        # 계산값이 직전 전송값 근처면 직전 값을 그대로 유지해서, 유한차분
        # 노이즈로 speed가 매 명령마다 바뀌는 것(=서보 프로파일 재계산 반복)을
        # 막는다. 상/하한에 걸린 경우는 그대로 통과시킴(클램프가 우선).
        if (
            self._last_sent_speed is not None
            and SPEED_MIN < speed < SPEED_MAX
            and abs(speed - self._last_sent_speed) < SPEED_HYSTERESIS
        ):
            speed = self._last_sent_speed

        return speed, delta_max, elapsed

    def listener_callback(self, msg):
        if self._released:
            return

        # [2026-07-29] 최대 20Hz로 send_angles 호출 빈도를 제한 — 100Hz
        # 그대로 보내면 펌웨어가 이전 가감속 프로파일을 끝내기 전에 매번
        # 재계산을 시작해서 "정지-출발-정지"처럼 끊겨 보임.
        now = self.get_clock().now()
        if (now - self._last_send_time).nanoseconds < THROTTLE_PERIOD_SEC * 1e9:
            return
        self._last_send_time = now

        # 创建字典将关节名称与位置值关联
        joint_state_dict = {name: msg.position[i] for i, name in enumerate(msg.name)}
        # 根据 RViz 顺序重新排列关节角度
        data_list = []
        for joint in self.rviz_order:
            # 获取弧度并转为角度
            if joint in joint_state_dict:
                radians_to_angles = round(math.degrees(joint_state_dict[joint]), 3)
                if joint == 'joint6output_to_joint6':
                    # 2026-07-24: 실물 서보 J6 회전 방향이 URDF 기준과 반대로 확인되어 부호 반전
                    radians_to_angles = -radians_to_angles
                data_list.append(radians_to_angles)

        # [2026-07-29] 직전 전송값과 거의 같으면(정지 상태) 재전송 생략 —
        # 불필요한 재가감속 재계산을 줄임.
        should_send = (
            self._last_sent_angles is None
            or len(data_list) != len(self._last_sent_angles)
            or any(
                abs(a - b) >= ANGLE_EPSILON_DEG
                for a, b in zip(data_list, self._last_sent_angles)
            )
        )
        if should_send:
            # [2026-07-30, TASK A] speed를 하드코딩(예전 40)하지 않고 실제
            # 각속도에서 역산 — PC 쪽 구간별 VELOCITY_SCALING이 무엇이든
            # 서보 추종 속도가 자동으로 따라옴.
            speed, delta_max, elapsed = self._servo_speed_for(data_list, now)
            # [2026-07-30, _async=True — relay 병목 근본 해결] pymycobot 소스
            # (mycobot280.py `_res`, common.py `read`) 실측 분석 결과:
            #
            # send_angles()는 has_reply=True를 주지 않아 펌웨어가 응답을 보내지
            # 않는데, 기본 경로(_async=False)의 `_res()`는 그걸 모르고
            #   while try_count < 3:  write() → read()  # wait_time=0.5s
            # 로 0.5초씩 3번 기다린 뒤 포기한다(MyCobot280은 crc_robot_class가
            # 아니라 Linux 기본 wait_time=0.5가 적용됨). 즉 호출 1회가 1.5초간
            # 블로킹되어, THROTTLE_PERIOD_SEC=0.05(20Hz)를 무의미하게 만들고
            # 실효 relay 주기를 0.64Hz로 떨어뜨렸다 — 로그 실측 dt=1.563s가
            # 3×0.5s와 일치(나머지 63ms는 write/직렬화 오버헤드).
            # 그 결과 명령당 이동량이 의도한 0.9°가 아니라 28°가 되어, 서보가
            # 28° 움직인 뒤 1.5초를 쉬는 것이 "구간 단위로 뚝뚝 끊김"의 정체였음.
            #
            # _async=True는 `_mesg`의 async 분기를 타서 write만 하고 즉시
            # 반환한다(읽기·재시도 없음). 20Hz 스트리밍 relay에는 이쪽이 맞음 —
            # 애초에 받을 응답이 없으므로 잃는 정보도 없다.
            #
            # 주의: async 경로는 `_res`가 매 호출 하던 reset_input_buffer()를
            # 건너뛴다. send_angles는 응답이 없어 정상적으로는 입력 버퍼에
            # 쌓일 것이 없고, get_angles()/get_fresh_mode() 같은 동기 호출이
            # 들어올 때 정리되지만, 장시간 구동 중 이상 동작이 보이면 입력
            # 버퍼 적체를 의심해볼 것.
            if delta_max is None:
                # 첫 전송이거나 refocus 직후(이력 리셋), 혹은 elapsed<=0.
                print('data_list: {} | speed={} (이력 없음)'.format(data_list, speed))
            else:
                # delta_max/elapsed는 SPEED_GAIN_K 재캘리브레이션용 실측 데이터.
                print(
                    'data_list: {} | speed={} (dmax={:.3f}deg, dt={:.3f}s, '
                    '{:.1f}deg/s)'.format(
                        data_list, speed, delta_max, elapsed, delta_max / elapsed
                    )
                )
            self.mc.send_angles(data_list, speed, _async=True)
            self._last_sent_angles = data_list
            self._last_sent_time = now
            self._last_sent_speed = speed

        # [2026-07-27] 그리퍼 relay 추가.
        if GRIPPER_JOINT_NAME in joint_state_dict:
            gripper_pos = joint_state_dict[GRIPPER_JOINT_NAME]
            flag = 0 if gripper_pos > GRIPPER_MIDPOINT else 1
            if flag != self._last_gripper_flag:
                print('gripper flag: {} (pos={:.3f})'.format(flag, gripper_pos))
                self.mc.set_gripper_state(flag, GRIPPER_SPEED, GRIPPER_TYPE)
                self._last_gripper_flag = flag


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()

    rclpy.spin(slider_subscriber)

    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
