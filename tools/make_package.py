#!/usr/bin/env python3
"""打离线教学包：本仓库 + 实验五/实验七源码（固定提交）+ 依赖清单。

教学镜像不一定能访问 GitHub，所以包里直接带上 src/experiment5 与
src/experiment7，并生成 BUNDLED_DEPENDENCIES.json（逐文件 sha256），
`tools/prepare_workspace.sh` 见到这个清单就只校验、不联网。

    python3 tools/make_package.py --out expPackages/experiment9_teaching_20261112_v4.zip
    python3 tools/make_package.py --deps-dir /tmp/dep9 --out ...   # 复用已有克隆

要求：本仓库工作区干净（清单按 HEAD 的内容生成）。
"""

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tarfile
import tempfile
import zipfile
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SKIP_DIRS = {'.git', '__pycache__', '.pytest_cache', 'build', 'install', 'log'}


def run(*cmd, cwd=None, capture=False):
    return subprocess.run(cmd, cwd=cwd, check=True, capture_output=capture)


def extract_archive(repo, revision, target):
    """把 git 里的某个提交导出到 target（等价 git archive | tar -x）。

    返回 {相对 target 的路径: 是否可执行}。权限位必须从 git 索引取：
    在 Windows 上 os.access(X_OK) 对任何文件都返回真，会把整包标成 755。

    archive 先落到临时文件再解开，不走管道：git 写、python 读的流式组合在
    Windows 上偶尔会两边互相等住（git 占着管道、tarfile 还没读完），落盘后
    git 的退出码也能直接检查。
    """
    target.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='git_archive_') as tmp:
        archive = Path(tmp) / 'snapshot.tar'
        subprocess.run(['git', '-C', str(repo), 'archive', '--format=tar',
                        '-o', str(archive), revision], check=True)
        with tarfile.open(archive, mode='r') as tar:
            try:
                tar.extractall(target, filter='tar')   # 保留权限位；Python 3.12 起默认过滤会改权限
            except TypeError:                          # 老版本没有 filter 参数
                tar.extractall(target)

    listing = subprocess.run(['git', '-C', str(repo), 'ls-tree', '-r', revision],
                             check=True, capture_output=True, text=True).stdout
    modes = {}
    for line in listing.splitlines():
        meta, _, path = line.partition('\t')
        if not path:
            continue
        modes[path] = meta.split()[0] == '100755'
    return modes


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def collect_files(base, root):
    """列出 base 下的普通文件，返回 {相对 root 的路径: sha256}。"""
    files = {}
    for path in sorted(base.rglob('*')):
        if any(part in SKIP_DIRS for part in path.relative_to(base).parts):
            continue
        if path.is_file():
            files[str(path.relative_to(root)).replace(os.sep, '/')] = sha256(path)
    return files


def force_script_modes(modes):
    """scripts/ 下的 .py/.sh 一律标成可执行。

    权限位在分发链路上不可靠：资源站转存、浏览器下载、Windows 解压都会把它抹成
    644，而 ``colcon build --symlink-install`` 装进 install/ 的是符号链接，权限跟着
    源码走，源码 644 时 ``ros2 launch`` 就报
    ``executable 'state_node.py' not found on the libexec directory``。
    上游 experiment5/experiment7 的 git 索引里本来就有几个脚本是 644（运行时用
    ``python3 x.py`` 调用），没必要让它们把整个链路带崩，这里统一抬到 755。
    """
    for path in list(modes):
        if '/scripts/' in path and path.endswith(('.py', '.sh')):
            modes[path] = True
        if path.startswith('tools/') and path.endswith(('.py', '.sh')):
            # Windows 上 core.fileMode=false，git 索引里新加的工具脚本一律是 644，
            # 这里补齐，免得同一目录里一半 755 一半 644。
            modes[path] = True
    return modes


def write_zip(stage, out, modes):
    out.parent.mkdir(parents=True, exist_ok=True)
    # 时间戳取打包时刻，不写死成 1980：解压到已经有 build/ 的工作区时，源文件的
    # mtime 必须比旧产物新，否则 make 跳过重编、cmake_install 跳过重装，install/
    # 里就留着上一版内容（典型症状：executable 'state_node.py' not found）。
    # 包内容的一致性由 BUNDLED_DEPENDENCIES.json 的逐文件 sha256 保证，不依赖
    # zip 字节可复现。
    stamp = datetime.now().timetuple()[:6]
    count = 0
    with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for path in sorted(stage.rglob('*')):
            relative = path.relative_to(stage)
            if any(part in SKIP_DIRS for part in relative.parts):
                continue
            if path.is_dir():
                info = zipfile.ZipInfo(str(relative).replace(os.sep, '/') + '/', stamp)
                info.external_attr = (0o40755 << 16) | 0x10
                zf.writestr(info, b'')
                continue
            relative_posix = str(relative).replace(os.sep, '/')
            info = zipfile.ZipInfo(relative_posix, stamp)
            mode = 0o100755 if modes.get(relative_posix) else 0o100644
            info.external_attr = mode << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            zf.writestr(info, path.read_bytes())
            count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', required=True, help='输出 zip 路径')
    parser.add_argument('--deps-dir', help='已克隆的 experiment5 / experiment7 所在目录')
    parser.add_argument('--allow-dirty', action='store_true', help='工作区有改动时仍然打包')
    args = parser.parse_args()

    out = Path(args.out).resolve()

    dirty = subprocess.run(['git', '-C', str(REPO_ROOT), 'status', '--porcelain'],
                           capture_output=True, text=True).stdout.strip()
    if dirty and not args.allow_dirty:
        print('工作区有未提交改动，先提交再打包（或用 --allow-dirty）：', file=sys.stderr)
        print(dirty, file=sys.stderr)
        return 2

    lock = json.loads((REPO_ROOT / 'SOURCE_LOCK.json').read_text(encoding='utf-8'))
    wanted = [item for item in lock['sources'] if item['name'] in ('experiment5', 'experiment7')]
    if len(wanted) != 2:
        print('SOURCE_LOCK.json 里找不到实验五/实验七的锁定记录', file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix='exp9_pkg_') as tmp:
        stage = Path(tmp) / 'stage'
        stage.mkdir()

        modes = extract_archive(REPO_ROOT, 'HEAD', stage)
        (stage / 'BUNDLED_DEPENDENCIES.json').unlink(missing_ok=True)

        manifest = {
            'source': f'实验10 教学包，依赖源码随包，{datetime.now():%Y-%m-%d} 打包',
            'dependencies': {},
        }

        for item in wanted:
            name, commit = item['name'], item['commit']
            local = Path(args.deps_dir) / name if args.deps_dir else None
            if local and (local / '.git').is_dir():
                head = run('git', '-C', str(local), 'rev-parse', 'HEAD',
                           capture=True).stdout.decode().strip()
                if head != commit:
                    print(f'{name} 的 HEAD 是 {head[:8]}，不是锁定的 {commit[:8]}', file=sys.stderr)
                    return 2
                source = local
            else:
                source = Path(tmp) / name
                if local:
                    print(f'{local} 不是 git 仓库，改为临时克隆', file=sys.stderr)
                run('git', 'clone', '--quiet', '--filter=blob:none', item['repository'], str(source))
                run('git', '-C', str(source), 'checkout', '--quiet', commit)

            target = stage / 'src' / name
            for path, executable in extract_archive(source, commit, target).items():
                modes[f'src/{name}/{path}'] = executable
            files = collect_files(target, stage)
            manifest['dependencies'][name] = {'commit': commit, 'files': files}
            print(f'{name} @ {commit[:8]}: {len(files)} 个文件已计入清单')

        (stage / 'BUNDLED_DEPENDENCIES.json').write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2) + '\n',
            encoding='utf-8', newline='\n')

        count = write_zip(stage, out, force_script_modes(modes))

    size = out.stat().st_size / 1024 / 1024
    print(f'已生成 {out}（{count} 个文件，{size:.1f} MB）')
    print('校验: 解压后执行 bash tools/prepare_workspace.sh')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
