#!/usr/bin/env python3
"""DB의 세션(map_id)별 노드 수와, 가장 최근 세션이 이전 세션들과 맺은 링크 수(중력 링크 제외).
사용: db_links.py <db>"""
import sys, sqlite3
db = sqlite3.connect(sys.argv[1])
mid = dict(db.execute('select id, map_id from Node').fetchall())
per = db.execute('select map_id, count(*) from Node group by map_id order by map_id').fetchall()
last = per[-1][0]
L = [(mid.get(a), mid.get(b), t) for a, b, t in db.execute('select from_id, to_id, type from Link') if t != 9]
new = [(a, b, t) for a, b, t in L if (a == last) != (b == last)]
to = {}
for a, b, t in new:
    o = b if a == last else a; to[o] = to.get(o, 0) + 1
print(f'세션별 노드: ' + ', '.join(f'map{m}={n}' for m, n in per))
print(f'새 세션 map{last} → 이전 세션 링크 {len(new)}개: ' + (', '.join(f'map{k} {v}' for k, v in sorted(to.items())) or '없음'))
