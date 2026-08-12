const mathJaxConfigScript = document.currentScript;

window.MathJax = {
  tex: {
    tags: "ams",
    packages: { "[+]": ["ams"] },
    inlineMath: [["\\(", "\\)"], ["$", "$"]],
    displayMath: [["\\[", "\\]"], ["$$", "$$"]],
    processEscapes: true,
    processEnvironments: true
  },
  options: {
    ignoreHtmlClass: ".*",
    processHtmlClass: "arithmatex"
  }
};

if (!document.querySelector("script[data-local-mathjax]")) {
  const mathJaxRuntime = document.createElement("script");
  mathJaxRuntime.src = new URL(
    "mathjax/tex-chtml-full.js",
    mathJaxConfigScript.src
  ).href;
  mathJaxRuntime.async = true;
  mathJaxRuntime.dataset.localMathjax = "true";
  document.head.appendChild(mathJaxRuntime);
}
