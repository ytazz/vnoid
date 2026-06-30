make_gettext_mofiles(CnoidVnoidJudgePlugin mofiles)

choreonoid_add_plugin(CnoidVnoidJudgePlugin
  VnoidJudgePlugin.cpp
  VnoidJudgeTargetDevice.cpp
  VnoidJudgeTargetItem.cpp
  VnoidJudgeItem.cpp
  VnoidJudgeFieldInfo.cpp
  ${mofiles}
  )

target_link_libraries(CnoidVnoidJudgePlugin PRIVATE CnoidBodyPlugin)
