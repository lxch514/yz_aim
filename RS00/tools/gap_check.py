#!/usr/bin/env python3
# 统计自动上报帧（心跳）的间隔分布，判断丢帧是总线丢的还是采集停顿时造成的；门限按实测中位间隔自适应
import re
import sys
from collections import Counter

PAT = re.compile(r'\+([\d.]+)s #(\d+)\] RX\s+ID=([0-9A-F]{8})')


def percentile(values, q):
    if not values:
        return 0.0
    data = sorted(values)
    k = min(len(data) - 1, max(0, int(round((len(data) - 1) * q))))
    return data[k]


def main(path):
    times = []
    with open(path, encoding='utf-8', errors='replace') as fp:
        for line in fp:
            m = PAT.search(line)
            if m and m.group(3).startswith('18'):
                times.append(float(m.group(1)))
    if len(times) < 10:
        print('心跳帧太少：{}'.format(len(times)))
        return 1

    gaps = [(b - a) * 1000.0 for a, b in zip(times, times[1:])]
    period = percentile(gaps, 0.5)
    lost = sum(max(0, int(round(g / period)) - 1) for g in gaps if g > 1.5 * period)
    expected = len(times) + lost

    print('心跳帧 {} 帧，跨 {:.1f}s，中位间隔 {:.2f}ms（设定周期以程序日志中的“目标周期”为准）'.format(
        len(times), times[-1] - times[0], period))
    print('按中位间隔推算：期望 {} 帧，估算丢失 {} 帧，丢帧率 {:.3f}%'.format(
        expected, lost, lost / expected * 100.0))
    print()
    print('间隔分布（按中位间隔 {:.1f}ms 分档）'.format(period))
    print('{:>20}  {:>8}  {:>8}  {}'.format('区间（ms）', '帧数', '占比', '说明'))
    buckets = [(0.0, 0.5, '挤在一起（同批读上来）'), (0.5, 0.8, '偏快'),
               (0.8, 1.3, '正常'), (1.3, 1.8, '略慢，未丢'),
               (1.8, 3.0, '丢 1 帧'), (3.0, 6.0, '丢 2~5 帧'),
               (6.0, 1e9, '丢 6 帧以上')]
    for lo, hi, note in buckets:
        cnt = sum(1 for g in gaps if lo * period <= g < hi * period)
        span = '{:.0f}~{:.0f}'.format(lo * period, min(hi * period, 99999))
        print('{:>20}  {:>8d}  {:>7.2f}%  {}'.format(
            span, cnt, cnt / len(gaps) * 100.0, note))
    print()
    print('百分位：p50 {:.2f}  p90 {:.2f}  p99 {:.2f}  p99.9 {:.2f}  max {:.2f}'.format(
        percentile(gaps, 0.5), percentile(gaps, 0.9), percentile(gaps, 0.99),
        percentile(gaps, 0.999), max(gaps)))

    # 大缺口前后各一个间隔的分布：后面紧跟很短的一批，说明那些帧是一起读上来的
    limit = 1.5 * period
    after = Counter()
    big = 0
    for i, gap in enumerate(gaps):
        if gap <= limit:
            continue
        big += 1
        nxt = gaps[i + 1] if i + 1 < len(gaps) else None
        if nxt is None:
            after['无后一间隔'] += 1
        elif nxt < 0.5 * period:
            after['后一间隔 < 0.5 倍中位（同批读上来）'] += 1
        elif nxt < 0.8 * period:
            after['后一间隔 0.5~0.8 倍中位'] += 1
        else:
            after['后一间隔 > 0.8 倍中位（孤立缺口）'] += 1
    print()
    print('超过 1.5 倍中位间隔（>{:.1f}ms）的缺口共 {} 个，其后一个间隔的分布：'.format(limit, big))
    for key, cnt in after.most_common():
        print('  {:<32} {}'.format(key, cnt))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
