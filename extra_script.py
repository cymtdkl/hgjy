"""
Clone Blue Pill 烧录修复脚本
=============================
国产 STM32F103 克隆芯片 (CS32/CKS32/APM32) 的 IDCODE 是 0x2ba01477,
而 OpenOCD 默认只认正版 STM32F103 的 0x1ba01477, 导致烧录失败。
此脚本在 OpenOCD 加载目标配置前注入 CPUTAPID 覆盖。
"""

Import("env")


def _fix_upload_flags():
    flags = env.get("UPLOADERFLAGS", [])
    new_flags = []
    inserted = False
    i = 0
    while i < len(flags):
        # 在 -f target/*.cfg 之前插入 CPUTAPID 设置
        if not inserted and flags[i] == "-f" and i + 1 < len(flags) \
                and "target" in flags[i + 1].lower():
            new_flags.append("-c")
            new_flags.append("set CPUTAPID 0x2ba01477")
            inserted = True
        new_flags.append(flags[i])
        i += 1

    if inserted:
        env.Replace(UPLOADERFLAGS=new_flags)
        print("  [CloneFix] CPUTAPID=0x2ba01477 已注入 (适配国产克隆芯片)")
    else:
        print("  [CloneFix] 警告: 未找到 target 配置, CPUTAPID 未注入")


_fix_upload_flags()
