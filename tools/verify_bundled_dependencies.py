#!/usr/bin/env python3
"""校验教学包里随包提供的依赖源码。

只在离线教学包里用：包根目录有 BUNDLED_DEPENDENCIES.json 时，说明
src/experiment5 与 src/experiment7 已经随包提供，此时不需要联网克隆，
只需确认它们与 SOURCE_LOCK.json 锁定的提交一致、且没有被改过。

    python3 tools/verify_bundled_dependencies.py [工作区]
"""

import hashlib
import json
import sys
from pathlib import Path


def main():
    root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[1]
    manifest_path = root / 'BUNDLED_DEPENDENCIES.json'
    if not manifest_path.is_file():
        print('没有 BUNDLED_DEPENDENCIES.json：这是代码仓库而不是教学包，')
        print('依赖请用 bash tools/prepare_workspace.sh 联网拉取。')
        return 2

    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    lock = json.loads((root / 'SOURCE_LOCK.json').read_text(encoding='utf-8'))
    locked = {item['name']: item for item in lock.get('sources', [])}

    for name, item in manifest['dependencies'].items():
        if name not in locked:
            raise RuntimeError(f'SOURCE_LOCK.json 里没有 {name}')
        if item['commit'] != locked[name]['commit']:
            raise RuntimeError(
                f'{name} 提交号与 SOURCE_LOCK.json 不一致: '
                f'{item["commit"]} != {locked[name]["commit"]}')
        base = (root / 'src' / name).resolve()
        if not base.is_dir():
            raise RuntimeError(f'缺少依赖目录: src/{name}')
        for relative, expected in item['files'].items():
            path = (root / relative).resolve()
            if base not in path.parents:
                raise RuntimeError(f'清单里有越界路径: {relative}')
            if not path.is_file():
                raise RuntimeError(f'缺少文件: {relative}')
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
            if actual != expected:
                raise RuntimeError(f'文件内容被改动: {relative}')
        print(f'{name}: {len(item["files"])} 个文件校验通过，提交 {item["commit"][:8]}')

    print('随包依赖就绪，下一步: bash tools/build.sh')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
