# SLAM Handbook 中文翻译

本目录包含已完成审查章节的 Markdown 翻译源稿、自动生成的 LaTeX 章节和中文 PDF。构建脚本会自动发现 `content/chapter-*.md`，因此后续章节经独立审查后只需加入同一目录，即可纳入 PDF 与网站构建。

```bash
python scripts/md_to_latex.py \
  --input content/chapter-1.md content/chapter-2.md ... content/chapter-18.md \
  --output latex/chapters/chapter-1.tex latex/chapters/chapter-2.tex ... latex/chapters/chapter-18.tex
```

Windows 本机构建需要 Python 3.10+、MiKTeX 或 TeX Live（包含 XeLaTeX），以及 MkDocs Material：

```powershell
winget install MiKTeX.MiKTeX
mpm --admin --install=ctex
python -m pip install mkdocs-material pymdown-extensions
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\build-windows.ps1
```

也可以只生成 PDF 而跳过网站：`.\scripts\build-windows.ps1 -SkipSite`。GitHub Actions 使用相同的 Windows 构建脚本，并在独立的 Pages 部署任务中发布静态站点。

网站为每个中文章节提供对应的 English original 页面和中英对照阅读页。对照页会在同一页面并列显示中文译文和英文原文，并按滚动进度同步；两个单语版本也可相互跳转。桌面端左右导航栏固定贴边、宽度稳定，并可独立收起；正文保留段落边界、首行缩进、图注和公式排版。翻译与审查规则见 `REQUIREMENTS.md`，术语统一规则见 `GLOSSARY.md`。

同步英文提取源：

```powershell
python scripts/sync_english_source.py --input <part-1.md> <part-2.md> <part-3.md> --output content\original\full.md --images-input <images-1> <images-2> <images-3> --images-output content\images
```

原著由 Cambridge University Press 出版。请保留原项目、作者、编辑和出版方信息，并仅在取得相应授权的范围内公开使用中文翻译。
