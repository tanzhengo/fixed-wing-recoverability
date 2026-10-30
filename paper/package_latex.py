#!/usr/bin/env python3
"""Package the editable single-file manuscript and its referenced assets."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    source = (root / 'main.tex').read_text()
    uncommented = re.sub(r'(?<!\\)%.*', '', source)
    if re.search(r'\\(?:input|include|bibliography|bibliographystyle)\s*\{', uncommented):
        raise ValueError('main.tex must contain all text, tables and references')
    assets = set(re.findall(r'\\includegraphics(?:\[[^\]]*\])?\{([^}]+)\}', uncommented))
    files = {'main.tex': (root / 'main.tex').read_bytes(),
             'IEEEtran.cls': (root / 'IEEEtran.cls').read_bytes(),
             'preview.pdf': (root / 'main.pdf').read_bytes()}
    for name in sorted(assets):
        path = (root / name).resolve()
        path.relative_to(root)
        files[name] = path.read_bytes()
    files['README.txt'] = '''单文件 LaTeX 论文编辑包

主文件：main.tex
推荐编译器：XeLaTeX

正文、公式、算法、表格、图注和参考文献均在 main.tex 中修改。
不需要在多个章节文件之间切换，不需要运行 BibTeX。
图片放在 figures/ 文件夹内；编辑时请保留该文件夹的相对位置。

Overleaf：上传本 ZIP，主文件选 main.tex，编译器选 XeLaTeX。
本地编辑器：解压全部文件，打开 main.tex，用 XeLaTeX 编译两次。
命令行也可使用 latexmk -xelatex main.tex 或 tectonic main.tex。

快速定位
- 搜索 BEGIN SECTION 定位各章节。
- 搜索 BEGIN REFERENCES 修改文末的 bibitem 参考文献条目。
- 搜索 label{fig: 或 label{tab: 定位图表。
- 在 author 命令中填写作者与单位，目前为匿名稿。

preview.pdf 为当前编译结果。SOURCE_MANIFEST.json 记录各文件校验值。
包内只有一个 .tex 文件；不依赖 ROS、Gazebo、Python 或实验数据。
IEEEtran.cls 保留原作者版权及 LPPL 声明。其余标准宏包由 LaTeX
环境提供，包括 amsmath、amssymb、graphicx、booktabs、array、url、
algorithm、algpseudocode、xurl、hyperref、fontenc。
'''.encode('utf-8')
    revision = subprocess.check_output(
        ['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = bool(subprocess.check_output(
        ['git', '-C', str(root), 'status', '--porcelain'], text=True).strip())
    manifest = {'source_commit': revision, 'source_worktree_dirty': dirty,
                'main_document': 'main.tex', 'tex_file_count': 1,
                'recommended_compiler': 'XeLaTeX', 'bibliography': 'embedded bibitems',
                'files_sha256': {name: hashlib.sha256(data).hexdigest()
                                 for name, data in sorted(files.items())}}
    files['SOURCE_MANIFEST.json'] = (json.dumps(manifest, indent=2) + '\n').encode()
    output = args.output.resolve()
    if output == root / 'main.tex' or output.suffix.lower() != '.zip':
        raise ValueError('Output must be a ZIP archive')
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'w', compression=zipfile.ZIP_DEFLATED,
                         compresslevel=9) as archive:
        for name, data in sorted(files.items()):
            archive.writestr(name, data)
    with zipfile.ZipFile(output) as archive:
        if archive.testzip() is not None:
            raise RuntimeError('ZIP integrity check failed')
    print(json.dumps({'archive': str(output), 'bytes': output.stat().st_size,
                      'tex_files': 1, 'figures': len(assets), 'files': len(files)}))


if __name__ == '__main__':
    main()
