# SLAM Handbook 第 1、2 章中文翻译

本目录包含第 1、2 章的 Markdown 翻译源稿、自动生成的 LaTeX 章节和中文 PDF。运行以下命令可重新生成章节源文件：

```bash
python scripts/md_to_latex.py \
  --input content/chapter-1.md content/chapter-2.md \
  --output latex/chapters/chapter-1.tex latex/chapters/chapter-2.tex
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

网站为每个中文章节提供对应的 English original 页面，两个版本可以相互跳转。桌面端左右导航栏固定贴边、宽度稳定，并可独立收起；正文保留段落边界、首行缩进、图注和公式排版。

同步英文提取源：

```powershell
python scripts/sync_english_source.py --input <full.md> --output content\original\full.md --images-input <images> --images-output content\images
```

原著由 Cambridge University Press 出版。请保留原项目、作者、编辑和出版方信息，并仅在取得相应授权的范围内公开使用中文翻译。
