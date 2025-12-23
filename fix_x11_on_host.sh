#!/bin/bash
# 快速 X11 修復腳本 - 在主機執行

echo "🔧 修復 Docker 容器的 X11 訪問權限..."
echo ""

# 允許本地 Docker 訪問 X11
xhost +local:docker

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ 已允許 Docker 訪問 X11"
    echo ""
    echo "現在可以在容器內執行："
    echo "   cd /workspace/koch_ros"
    echo "   python3 koch_mirror_sim_calibrated.py"
else
    echo ""
    echo "❌ 命令失敗，請確認："
    echo "   1. 你在主機上執行此腳本（不是容器內）"
    echo "   2. X11 server 正在運行"
    echo "   3. 已安裝 xhost 命令"
fi
