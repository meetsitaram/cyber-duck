"""Run one script in Fusion through its own MCP server, for a session that doesn't have the server loaded as a tool.

  python -I tools/fusion_call.py script.py [--write] [--recipe NAME]

Read-only unless --write. --recipe sets RECIPE = '...' in the script. Prints the tool's output.
"""
import json
import os
import re
import sys
import urllib.request

URL = 'http://127.0.0.1:27182/mcp'
HEADERS = {'Content-Type': 'application/json', 'Accept': 'application/json, text/event-stream'}


def post(payload, session=None):
    h = dict(HEADERS)
    if session:
        h['MCP-Session-Id'] = session
    req = urllib.request.Request(URL, data=json.dumps(payload).encode('utf-8'), headers=h, method='POST')
    with urllib.request.urlopen(req, timeout=2400) as r:
        body, sid = r.read().decode('utf-8').strip(), r.headers.get('MCP-Session-Id')
    if body.startswith(('event:', 'data:')):
        body = [x[5:].strip() for x in body.splitlines() if x.startswith('data:')][-1]
    return (json.loads(body) if body else None), sid


def main():
    script = open(sys.argv[1], encoding='utf-8').read()
    if '--recipe' in sys.argv:
        script = re.sub(r"^RECIPE = '.*'$", f"RECIPE = '{sys.argv[sys.argv.index('--recipe') + 1]}'", script, flags=re.M)
    # this checkout's code, not whichever one Fusion's Python imported last: several worktrees can share one Fusion
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    script = script.replace('C:\\Users\\meets\\Projects\\duck-droids', root)
    script = script.replace('if P not in sys.path:\n    sys.path.insert(0, P)', "sys.path[:] = [x for x in sys.path if not x.endswith(os.path.join('cad', 'fusion'))]\nsys.path.insert(0, P)")
    _, s = post({'jsonrpc': '2.0', 'id': 1, 'method': 'initialize', 'params': {'protocolVersion': '2025-06-18', 'capabilities': {}, 'clientInfo': {'name': 'duck-droids', 'version': '1'}}})
    post({'jsonrpc': '2.0', 'method': 'notifications/initialized'}, s)
    obj = {'script': script}
    if '--write' not in sys.argv:
        obj['readOnly'] = True
    r, _ = post({'jsonrpc': '2.0', 'id': 2, 'method': 'tools/call', 'params': {'name': 'fusion_mcp_execute', 'arguments': {'featureType': 'script', 'object': obj}}}, s)
    for part in ((r or {}).get('result') or {}).get('content', []):
        t = part.get('text', str(part))
        try:
            j = json.loads(t)
            t = j.get('message') or j.get('error') or t
        except ValueError:
            pass
        print(t)
    if 'error' in (r or {}):
        print('RPC error:', r['error'])


if __name__ == '__main__':
    main()
