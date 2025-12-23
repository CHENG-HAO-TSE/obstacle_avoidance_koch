#!/bin/bash
# X11 修復腳本 - 用於 Docker 容器中的 GUI 應用

echo "🔧 修復 X11 權限..."

# 設置 DISPLAY
export DISPLAY=${DISPLAY:-:1}

# 方法 1: 從主機用戶的 Xauthority 讀取
# Docker 容器應該用 -v /home/USER/.Xauthority:/root/.Xauthority:ro 掛載
if [ -f /root/.Xauthority ]; then
    export XAUTHORITY=/root/.Xauthority
    echo "   使用已掛載的 Xauthority"
elif [ -f ~/.Xauthority ]; then
    export XAUTHORITY=~/.Xauthority
    echo "   使用本地 Xauthority"
else
    # 方法 2: 嘗試從 /tmp 尋找
    if [ -f /tmp/.docker.xauth ]; then
        export XAUTHORITY=/tmp/.docker.xauth
        echo "   使用 /tmp/.docker.xauth"
    else
        # 方法 3: 創建新的（可能不會工作）
        touch ~/.Xauthority
        chmod 600 ~/.Xauthority
        export XAUTHORITY=~/.Xauthority
        echo "   ⚠️  創建新的 Xauthority（可能需要主機授權）"
    fi
fi

# 設置環境變數以改善兼容性
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1

# 測試 X11 連接
if command -v xdpyinfo >/dev/null 2>&1; then
    if xdpyinfo >/dev/null 2>&1; then
        echo "✅ X11 連接成功"
    else
        echo "❌ X11 連接失敗 - 將使用無 GUI 模式"
        echo ""
        echo "💡 解決方法："
        echo "   1. 在主機執行: xhost +local:docker"
        echo "   2. 或重新啟動容器時加上:"
        echo "      docker run -v /tmp/.X11-unix:/tmp/.X11-unix \\"
        echo "                 -v ~/.Xauthority:/root/.Xauthority:ro \\"
        echo "                 -e DISPLAY=$DISPLAY ..."
    fi
else
    echo "   跳過 X11 測試（xdpyinfo 不可用）"
fi

echo ""
echo "環境變數:"
echo "   DISPLAY=$DISPLAY"
echo "   XAUTHORITY=$XAUTHORITY"
echo ""

# 如果提供了命令參數，執行它
if [ $# -gt 0 ]; then
    echo "▶️  執行: $@"
    exec "$@"
fi
