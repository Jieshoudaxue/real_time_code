#!/bin/bash

cleanup() {
    echo "Cleaning up..."
    rm -f test
    exit 1 # 如果是由于接收到信号而调用 cleanup，则退出状态不应该为0
}

# 设置 trap，当捕捉到 SIGINT 信号时调用 cleanup 函数
trap 'cleanup' SIGINT

# 检查是否提供了文件名
if [ -z "$1" ]; then
    echo "Usage: $0 <source_file>"
    exit 1
fi

# 检查文件是否存在
if [ ! -f "$1" ]; then
    echo "File $1 does not exist."
    exit 1
fi

# 编译源文件
if ! g++ -std=c++11 "$1" -pthread -o test; then
    echo "Compilation failed."
    exit 1
fi

# 运行编译后的程序
./test
run_status=$?

# 清理生成的可执行文件
cleanup

# 退出并返回程序的退出状态
exit $run_status
