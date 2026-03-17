#!/usr/bin/env python3
"""
Reads per-scenario JSON results and generates a markdown report table.
"""

import json
import os

RESULTS_DIR = '/results'

SCENARIOS = [
    ('1_udp_baseline',           'UDP',  'No',  '0%', 'No'),
    ('2_tcp_baseline',           'TCP',  'No',  '0%', 'No'),
    ('3_udp_compress',           'UDP',  'Yes', '0%', 'No'),
    ('4_tcp_compress',           'TCP',  'Yes', '0%', 'No'),
    ('5_udp_loss2',              'UDP',  'No',  '2%', 'No'),
    ('6_udp_compress_loss2',     'UDP',  'Yes', '2%', 'No'),
    ('7_udp_loss2_fec',          'UDP',  'No',  '2%', 'Yes'),
    ('8_udp_compress_loss2_fec', 'UDP',  'Yes', '2%', 'Yes'),
]

lines = []
lines.append('# Nimbro Topic Transport — E2E Test Report\n')
lines.append('')
lines.append('| # | Proto | Compress | Loss | FEC | Msgs | Rate (Hz) | Seq Loss % | RX BW (B/s) | Drop Rate | Noise | Pass |')
lines.append('|---|-------|----------|------|-----|------|-----------|------------|-------------|-----------|-------|------|')

for name, proto, compress, loss, fec in SCENARIOS:
    json_path = os.path.join(RESULTS_DIR, f'{name}.json')

    if not os.path.exists(json_path):
        lines.append(f'| {name} | {proto} | {compress} | {loss} | {fec} | - | - | - | - | - | - | SKIP |')
        continue

    with open(json_path) as f:
        r = json.load(f)

    msgs = r.get('msg_count', 0)
    rate = r.get('rate_hz', 0)
    seq_loss = r.get('seq_loss_pct', 0)
    noise = r.get('noise_leaked', 0)
    passed = 'PASS' if r.get('pass', False) else 'FAIL'

    stats = r.get('receiver_stats') or {}
    bw = stats.get('bandwidth', 0)
    bw_str = f'{bw:,.0f}' if bw else '-'
    drop = stats.get('drop_rate', 0)
    drop_str = f'{drop:.2%}' if drop is not None else '-'

    lines.append(
        f'| {name} | {proto} | {compress} | {loss} | {fec} '
        f'| {msgs} | {rate} | {seq_loss}% | {bw_str} | {drop_str} | {noise} | {passed} |'
    )

lines.append('')
lines.append('## Legend')
lines.append('- **Msgs**: Total messages received in measurement window')
lines.append('- **Rate**: Effective receive rate (UDP with rate limit: expect ~50 Hz, TCP: up to 100 Hz)')
lines.append('- **Seq Loss %**: Percentage of sequence numbers missing in received range')
lines.append('- **RX BW**: Receiver bandwidth from nimbro /network/receiver_stats (bytes/sec)')
lines.append('- **Drop Rate**: Packet drop rate from nimbro receiver stats (UDP only)')
lines.append('- **Noise**: Noise topic messages that leaked through (should be 0)')
lines.append('')

report = '\n'.join(lines)
print(report)

with open(os.path.join(RESULTS_DIR, 'report.md'), 'w') as f:
    f.write(report)
