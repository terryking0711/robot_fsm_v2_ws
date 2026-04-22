#pragma once

enum class Stage3State
{
  S3_ENTER,         // 進入第三關
  S3_LOCALIZE,      // 確認定位資訊
  S3_SCAN_HAY,      // 接收視覺結果
  S3_PLAN_STACK,    // 規劃堆疊
  S3_SELECT_TARGET, // 選擇目標稻草卷
  S3_NAV_TO_PICK,   // 導航到抓取位置
  S3_PICK_HAY,      // 發命令抓取
  S3_NAV_TO_STACK,  // 導航到堆疊位置
  S3_PLACE_HAY,     // 發命令放置
  S3_VERIFY_STABLE, // 確認穩定
  S3_DONE,          // 第三關完成
  S3_FAILED         // 第三關失敗
};
