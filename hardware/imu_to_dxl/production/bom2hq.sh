#!/bin/sh
# 用法:
#   ./fix_bom_designators.sh "bom（HQ）.csv" > "bom_new.csv"
# 或:
#   ./fix_bom_designators.sh "bom（HQ）.csv" "bom_new.csv"

set -eu

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "用法: $0 input.csv [output.csv]" >&2
    exit 1
fi

input=$1
output=${2:-}

if [ -n "$output" ] && [ "$input" = "$output" ]; then
    echo "错误: 输入和输出不能是同一个文件。请先输出到新文件，再替换。" >&2
    exit 1
fi

run_awk() {
    awk '
    BEGIN { FS=","; OFS="," }

    NR == 1 {
        sub(/^\357\273\277/, "", $0)   # 去掉 UTF-8 BOM
        print
        next
    }

    {
        line = $0

        # 数据行第一列形如 "C1_10, C1_9, ..., C1",...
        if (match(line, /^"[^"]*",/)) {
            first = substr(line, 2, RLENGTH - 3)  # 取出引号内的第一列
            rest  = substr(line, RLENGTH + 1)     # 第一列后面的其余列

            n = split(first, arr, ",")
            out = ""
            split("", seen)                       # 清空去重数组

            for (i = 1; i <= n; i++) {
                t = arr[i]
                gsub(/^ +| +$/, "", t)             # 去掉 token 两端空格
                sub(/_[0-9]+$/, "", t)             # 去掉 _10、_9、...、_2 后缀

                if (t == "" || seen[t]++) continue
                out = (out == "" ? t : out ", " t)
            }

            print "\"" out "\"," rest
        } else {
            print
        }
    }
    ' "$input"
}

if [ -n "$output" ]; then
    run_awk > "$output"
else
    run_awk
fi
