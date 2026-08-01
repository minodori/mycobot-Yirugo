# 수확 시퀀스 재 검토 
- 현재 코드는 ai와 여러번 수정해서 복잡해졌다. 문제가 있는 경우 제한, 필터링, 리팩토 등을 거쳐 사람이 이해하기가 점점 힘들어지고 있다. 따라서 근본적인 문제에 대한 해결책을 찾고자 한다.
- 문제의 원인은 모든 시작이  look pos이고 종료도 look pos란 점이다.
- 실제 시나리오에서 수확통도 빠져있다. 실제 시나리오대로 구성한다.
- armed pos를 추가한다. 모든 수확의 시작과 종료는 armed pos이다.
- look pos는 관측시에만 이용한다.
- 수확할 타멧을 모두 플래닝해서 IK해가 가장 잘 풀릴 수 있는 위치로 이동(armed pos)
    - moveit2는 rrt 기반이라 램덤하게 찾은 해를 보내주는 방식
    - 이 해를 바로 실행하지말고 일정 수준(e.g. 10개)를 받늗다
    - 다음 각관절의 이동이 최소이거나 관절이 많이 꼬이는 경로를 배제한 최적 경로를 선택한다. 그 경로의 시작점이 armed pos로 정의한다.
- armed pos
    - 현재는 look pos에서 1/5~5/5단계 반복 진행한다.
    - 매번 look pos에서 다양한 타겟으로 이동하므로 moveit2 경로계획이 들쭉 날쭉하다.
    - 옵션1 :armed pos - 타겟 - armed pos - 수확통 - armed pos -
    - 옵션2 : 각 타겟 위치별 armed pos 결정(IK해가 가장 잘 풀리는) 한 후 armed pos - target - 수확통 - armed pos -  이 경우 타겟별로 armed pos가 다르다.

- 또하나 문제점은 python pymycobot api send_coords의 좌표와 현재 이 코드의 좌표 기준이 다르단 것이다
    - send_coords는 g-base - j6 flange 중심인데 우리는 g-base - gripper 끝단(9cm) 이다. 
    - gripper는 기본 고정 부분이 아니므로 j6 flange 로 하는게 맞을 것 같다. send_coords와 호환성도 고려해야된다.

- armed pos - 수확통 - arme pos
    - 수확통은 g_base 기준 0,y,z에 있다. 이 값은 고정이다.
    - 그래서 armed pos 에서 j1만 반시계 방향으로 90도 회전하면 도달한다.
    - 반경은 조정이 필요하다.(y) 


