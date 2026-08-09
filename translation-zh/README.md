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
python -m pip install mkdocs-material pymdown-extensions
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\build-windows.ps1
```

也可以只生成 PDF 而跳过网站：`.\scripts\build-windows.ps1 -SkipSite`。GitHub Actions 使用相同的 Windows 构建脚本，并在独立的 Pages 部署任务中发布静态站点。

原著由 Cambridge University Press 出版。请保留原项目、作者、编辑和出版方信息，并仅在取得相应授权的范围内公开使用中文翻译。
