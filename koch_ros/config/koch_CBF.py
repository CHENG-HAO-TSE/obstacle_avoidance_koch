#!/usr/bin/env python3
import time, math, numpy as np
import pybullet as p, pybullet_data
import torch

# ================== 基本設定 ==================
USE_GUI = True
# 我們現在只使用幾何 CBF，因此只有一個安全半徑
SAFE_RADIUS_M = 0.10 # Koch 降低安全範圍，允許更接近障礙物

# *** 關鍵修改：定義要保護的關鍵連桿 ***
# Koch arm: 保護後段連桿 link3_1(2), link4_1(3), gripper_static_1(4), gripper_moving_1(5)
CRITICAL_LINKS = [2, 3, 4, 5]

MAX_EE_VEL = 0.2
Kp, STEP_GAIN, VEL_CLAMP = 0.5, 0.06, 0.8
goal = np.array([0.15, 0.00, 0.15], dtype=np.float32)  # Koch 手臂較短，目標更近
BALL_RADIUS = 0.03  # 障礙物球體縮小
EE_LINK = 5 # Koch arm: gripper_moving_1 作為末端執行器
N_J = 6 # Koch arm 有 6 個可動關節
# CKPT = "sdf_256x5_mesh.pt" # 不再需要
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
DEBUG_CBF = True

print(f"[INFO] Using device: {DEVICE}")
print(f"[INFO] 🤖 機器人: Koch Arm (6-DOF 機械臂)")
print(f"[INFO] ⚠️  已切換至「多點幾何 CBF」系統 (放棄 JSDF)。")
print(f"[INFO] 正在保護連桿: {CRITICAL_LINKS}")
print(f"[INFO] 統一安全半徑: {SAFE_RADIUS_M}m")


# ================== JSDF: 載入與計算 ==================
# *** 關鍵修改：完全移除所有 JSDF 相關函數 ***
# (load_jsdf, jsdf_min_dist_only, jsdf_min_d_and_grad 已被刪除)


# ================== CBF 增益 ==================
CBF_GAIN_MIN, CBF_GAIN_MAX = 1.5, 6.0
def adaptive_cbf_gain(dist, safe_r):
    if dist > 2.5*safe_r: return CBF_GAIN_MIN
    if dist < 1.1*safe_r: return CBF_GAIN_MAX
    ratio = (dist - 1.1*safe_r)/(1.4*safe_r)
    return CBF_GAIN_MIN + (CBF_GAIN_MAX - CBF_GAIN_MIN)*(1 - ratio)

# ================== PyBullet 場景 ==================
def setup_pybullet(use_gui=True):
    cid = p.connect(p.GUI if use_gui else p.DIRECT)
    if cid < 0: raise RuntimeError("Failed to connect PyBullet.")
    print(f"PyBullet 連接 ID: {cid}")
    if use_gui:
        print("啟用 GUI 模式...")
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        # Koch arm 較小，相機更近以便觀察
        p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=50, cameraPitch=-20, cameraTargetPosition=[0.15, 0.05, 0.15])
        print("[INFO] 相機位置已設定：距離=0.8m, 偏航=50°, 俯仰=-20°, 目標=[0.15,0.05,0.15]")
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.setRealTimeSimulation(0)
    plane = p.loadURDF("plane.urdf")
    print(f"[INFO] Plane loaded with ID: {plane}")
    robot = p.loadURDF("/workspace/model/koch_arm/koch.urdf", useFixedBase=True)
    print(f"[INFO] Robot loaded with ID: {robot}")
    return robot

robot = setup_pybullet(USE_GUI)

# joints
J_IDX_MOVABLE = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED]
TOTAL_DOF = len(J_IDX_MOVABLE)
if TOTAL_DOF < N_J: raise RuntimeError(f"Robot DOF ({TOTAL_DOF}) < expected {N_J}.")
print(f"[INFO] 偵測到 {TOTAL_DOF} 個可動關節。")
J_IDX = list(range(N_J))  # Koch: [0,1,2,3,4,5] (joint1~joint5 + joint_gripper)

# 讀取關節限制
j_lo = np.array([p.getJointInfo(robot, j)[8] for j in J_IDX])
j_hi = np.array([p.getJointInfo(robot, j)[9] for j in J_IDX])

# 顯示關節限制
print("\n[INFO] 🔧 關節限制 (rad):")
joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint_gripper']
for name, lo, hi in zip(joint_names, j_lo, j_hi):
    print(f"  {name:15s}: [{lo:6.3f}, {hi:6.3f}] rad = [{np.degrees(lo):6.1f}°, {np.degrees(hi):6.1f}°]")

# Koch arm 的安全初始位置 (在關節限制內)
q_home = [0.0, 1.0, 1.5, 0.0, 0.0, 0.5]
q_home = np.clip(q_home, j_lo, j_hi)  # 確保在限制內
print(f"\n[INFO] 🏠 初始位置: {[f'{q:.3f}' for q in q_home]}")

for j, q0 in zip(J_IDX, q_home): p.resetJointState(robot, j, q0)

# balls
def create_ball(color=(1,0,0,1), r=BALL_RADIUS, pos=(0.5,0,0.35)):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=color)
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=r)
    ball_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis, basePosition=pos)
    print(f"[INFO] Ball created with ID: {ball_id} at position {pos}, color {color}, radius {r}")
    return ball_id
ball1 = create_ball(color=(1,0,0,1), pos=(0.18, 0.00, 0.18))
ball2 = create_ball(color=(0,0,1,1), pos=(0.22, 0.08, 0.16))

def centers_fn(t):
    A, v = 0.08, 0.10  # Koch 較小，振幅和速度更小
    w = v/A
    c1 = np.array([0.18, -0.10, 0.20 + A*math.sin(w*t)], dtype=np.float32)
    c2 = np.array([0.18,  0.10, 0.20 + A*math.sin(w*t+math.pi)], dtype=np.float32)
    return [c1, c2]

def upd_balls(cs):
    p.resetBasePositionAndOrientation(ball1, cs[0].tolist(), [0,0,0,1])
    p.resetBasePositionAndOrientation(ball2, cs[1].tolist(), [0,0,0,1])

# ================== 幾何計算輔助函數 ==================

def get_link_pos(link_idx):
    """獲取指定連桿的原點位置"""
    return np.array(p.getLinkState(robot, link_idx)[4])

def get_link_jacobian(q_arm, link_idx):
    """獲取指定連桿原點的雅可比矩陣"""
    q_full = list(q_arm) + [0.0]*(TOTAL_DOF - N_J)
    z = [0.0]*TOTAL_DOF
    Jlin = p.calculateJacobian(robot, link_idx, [0,0,0], q_full, z, z)[0]
    return np.array(Jlin)[:, :N_J]  # (3,7)

# ================== QP-IK ==================
USE_QP = True
try:
    import cvxpy as cp
    print(f"[INFO] CVXPY 已載入，使用 QP 求解器")
except Exception as e:
    USE_QP = False
    cp = None
    print(f"[WARN] CVXPY 未安裝，使用投影方法：{e}")

# *** 修改：QP 求解器現在只接受 d, g, r 列表 ***
def solve_qp(J_task, v_des, q, d_list, g_list, r_list):
    dq = cp.Variable(N_J)
    cons = [dq >= -VEL_CLAMP, dq <= VEL_CLAMP]
    q_next = q + STEP_GAIN*dq
    cons += [q_next >= j_lo, q_next <= j_hi]
    
    for d, g, r in zip(d_list, g_list, r_list):
        if np.linalg.norm(g) < 1e-6: continue
        alpha = adaptive_cbf_gain(d, r)
        h = d - r
        cons.append( -g @ dq <= alpha*h )
        
    # 任務目標：最小化末端執行器速度誤差
    prob = cp.Problem(cp.Minimize(cp.sum_squares(J_task @ dq - v_des)), cons)
    
    try:
        prob.solve(solver=cp.OSQP, warm_start=True, max_iter=5000, eps_abs=1e-4, eps_rel=1e-4, verbose=False, polish=True)
    except cp.error.SolverError:
        print("[WARN] OSQP 求解器失敗。")
        prob.status = "solver_error"

    if prob.status not in ("optimal","optimal_inaccurate") or dq.value is None:
        dq_ls = np.clip(np.linalg.pinv(J_task) @ v_des, -VEL_CLAMP, VEL_CLAMP)
        return dq_ls
    return dq.value

def projective_step(J_task, v_des, q, d_list, g_list, r_list):
    dq = np.clip(np.linalg.pinv(J_task) @ v_des, -VEL_CLAMP, VEL_CLAMP)
    worst_viol, worst_g = 0.0, None
    for d, g, r in zip(d_list, g_list, r_list):
        h = d - r
        viol = -g.dot(dq) - adaptive_cbf_gain(d, r)*h
        if viol > worst_viol: worst_viol, worst_g = viol, g
    if worst_g is not None and worst_viol > 0:
        dq = dq + (worst_viol/(np.linalg.norm(worst_g)**2 + 1e-12))*worst_g
    dq = np.clip(dq, -VEL_CLAMP, VEL_CLAMP)
    q_tmp = np.clip(q + STEP_GAIN*dq, j_lo, j_hi)
    return (q_tmp - q)/STEP_GAIN

# 目標球可視化
goal_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0,1,0,0.7])
goal_body = p.createMultiBody(baseMass=0, baseVisualShapeIndex=goal_vis, basePosition=goal.tolist())
print(f"[INFO] Goal ball created with ID: {goal_body} at position {goal.tolist()}")

print(f"[INFO] GUI={USE_GUI} | QP={'ON' if USE_QP else 'OFF'}")
print("[INFO] === 模擬開始 ===")

# ================== 主迴圈 ==================
t0 = time.time()
try:
    it = 0
    while p.isConnected():
        t = time.time() - t0
        q = np.array([p.getJointState(robot, j)[0] for j in J_IDX], dtype=np.float64)
        
        # 1. 任務：獲取末端執行器 (EE_LINK) 的狀態
        x_ee = get_link_pos(EE_LINK)
        J_ee = get_link_jacobian(q, EE_LINK) # 任務雅可比 J_task

        v_des = Kp*(goal - x_ee)
        vn = np.linalg.norm(v_des)
        if vn > MAX_EE_VEL: v_des *= (MAX_EE_VEL/vn)

        centers = centers_fn(t); upd_balls(centers)
        
        # *** 關鍵修改 (3) - 建立多點幾何約束 ***
        
        all_d = []
        all_g = []
        all_r = []
        link_states = {} # 緩存連桿狀態以供日誌使用

        for link_idx in CRITICAL_LINKS:
            # 獲取該連桿的狀態和雅可比
            x_link = get_link_pos(link_idx)
            J_link = get_link_jacobian(q, link_idx) # 約束雅可比 J_constraint
            link_states[link_idx] = x_link # 儲存位置
            
            for center in centers:
                dist_vec = x_link - center # 從障礙物指向連桿
                dist = np.linalg.norm(dist_vec)
                
                # 計算梯度 g = (grad_x_d) * J_link
                grad_x_d = dist_vec / (dist + 1e-9) # (3,)
                grad_q_d = grad_x_d @ J_link        # (3,) @ (3,7) -> (7,)
                
                all_d.append(dist)
                all_g.append(grad_q_d)
                all_r.append(SAFE_RADIUS_M) # 所有點都用同一個安全半徑

        # 3. 求解
        if USE_QP:
            # J_ee 是任務，(all_d, all_g, all_r) 是約束
            dq = solve_qp(J_ee, v_des, q, all_d, all_g, all_r)
        else:
            dq = projective_step(J_ee, v_des, q, all_d, all_g, all_r)
            
        q_cmd = q + STEP_GAIN*dq

        # 4. 執行 - Koch arm 控制所有 6 個關節
        for j in range(N_J):
            p.setJointMotorControl2(robot, J_IDX[j], p.POSITION_CONTROL, targetPosition=float(q_cmd[j]), force=87.0)

        # 檢查實際碰撞
        contact_pts = p.getContactPoints(robot)
        has_collision = len(contact_pts) > 0
        
        if it % 60 == 0:  # 每 0.25 秒打印一次
            
            min_d_overall = min(all_d) if all_d else float('inf')
            cbf_active = min_d_overall < SAFE_RADIUS_M
            
            collision_mark = "🔴 COLLISION!" if has_collision else ""
            print(f"[t={t:5.2f}s] Min_Geo_d={min_d_overall:.3f}m | " + 
                  f"CBF={'ACTIVE' if cbf_active else 'idle'} | {collision_mark}")
            
            if DEBUG_CBF:
                # 打印每個受監控連桿的最小距離
                log_msg = "  [Dist] "
                for link_idx in CRITICAL_LINKS:
                    min_d_link = float('inf')
                    for center in centers:
                        d = np.linalg.norm(link_states[link_idx] - center)
                        if d < min_d_link: min_d_link = d
                    log_msg += f"L{link_idx}={min_d_link:.3f}m | "
                print(log_msg)

        if has_collision and it % 10 == 0:
            print(f"  ⚠️  [WARNING] 檢測到碰撞！接觸點數量: {len(contact_pts)}")
            for contact_point in contact_pts[:3]:
                link_a, link_b = contact_point[3], contact_point[4]
                print(f"      連桿{link_a} <-> 物體{link_b}")
        
        it += 1
        p.stepSimulation()

except KeyboardInterrupt:
    print("\n[INFO] Stopped by user.")
except p.error as e:
    print(f"\n[PYBULLET ERROR] PyBullet 崩潰: {e}")
    print("這通常意味著 GUI 視窗被關閉或 X11 連線中斷。")
finally:
    if p.isConnected():
        p.disconnect()
    print("[INFO] === 模擬結束 ===")