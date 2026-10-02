# 機構 STM 回報的第三關自動夾取狀態。
# topic: /mechanism/straw_pick_status，STM 以 10~20 Hz 持續發送（不是只在變化時發）。
# 全部固定長度，micro-ROS 端不需要配置字串 / 陣列 buffer。
#
# STM 端行為約定：
#   - /mechanism/command id=320 -> armed = true ; id=321 -> armed = false（開機預設 false）
#   - 只有 armed 時，前方極限開關按下才執行夾取腳本
#   - 腳本執行期間 phase = PICKING，完成後 pick_count++ 並回到 IDLE
#   - 開關必須先放開，才能再觸發下一次夾取

uint8 PHASE_IDLE = 0
uint8 PHASE_PICKING = 1
uint8 PHASE_ERROR = 2

uint8 phase           # PHASE_*
uint8 pick_count      # 開機以來夾取完成的累計數（只增不減）
bool armed            # 目前是否允許「碰到開關就自動夾取」
bool switch_pressed   # 前方極限開關原始狀態
