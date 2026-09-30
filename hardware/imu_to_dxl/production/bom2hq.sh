#!/bin/sh
# 用法:
#   ./fix_bom.sh "bom（HQ）.csv" > "bom_new.csv"
# 或:
#   ./fix_bom.sh "bom（HQ）.csv" "bom_new.csv"

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
    BEGIN {
        FS = ","
        OFS = ","
    }

    NR == 1 {
        sub(/^\357\273\277/, "", $0)   # 去掉 UTF-8 BOM
        print
        next
    }

    {
        line = $0

        # 数据行第一列形如: "C1_10, C1_9, ..., C1",C0402,10,...
        if (match(line, /^"[^"]*",/)) {
            first = substr(line, 2, RLENGTH - 3)   # 第一列引号内的内容
            rest  = substr(line, RLENGTH + 1)      # 第一列后面的内容，如 C0402,10,100p,C106200

            n = split(first, arr, ",")
            out = ""
            count = 0
            split("", seen)                        # 清空去重数组

            for (i = 1; i <= n; i++) {
                t = arr[i]
                gsub(/^[ \t]+|[ \t]+$/, "", t)     # 去掉两端空格
                sub(/_[0-9]+$/, "", t)             # 去掉 _10、_9、...、_2 后缀

                if (t == "" || seen[t]++) continue

                out = (out == "" ? t : out ", " t)
                count++
            }

            # 解析 rest: Footprint,Quantity,Value,LCSC...
            p1 = index(rest, ",")
            if (p1 == 0) {
                print line
                next
            }

            footprint = substr(rest, 1, p1 - 1)
            rest2 = substr(rest, p1 + 1)

            p2 = index(rest2, ",")
            if (p2 == 0) {
                tail = ""
            } else {
                tail = substr(rest2, p2 + 1)
            }

            # 输出: 去重后的位号, 封装, 去重后的数量, 其余列
            if (tail == "") {
                print "\"" out "\"," footprint "," count
            } else {
                print "\"" out "\"," footprint "," count "," tail
            }
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
