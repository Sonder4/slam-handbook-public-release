(() => {
  const embedded = new URLSearchParams(window.location.search).has("embed");
  let bilingualController;

  function documentReady(callback) {
    if (typeof document$ !== "undefined") {
      document$.subscribe(callback);
    } else {
      document.addEventListener("DOMContentLoaded", callback);
    }
  }

  function initializeEmbed() {
    document.body.classList.add("reader-embed");
  }

  function initializeBilingualReader() {
    const toggle = document.querySelector("[data-bilingual-toggle]");
    const content = document.querySelector(".md-content__inner");
    if (!toggle || !content) {
      bilingualController?.abort();
      bilingualController = undefined;
      document.body.classList.remove("bilingual-reader-page");
      return;
    }
    if (toggle.dataset.bilingualInitialized === "true") return;

    bilingualController?.abort();
    bilingualController = new AbortController();
    const { signal } = bilingualController;
    toggle.dataset.bilingualInitialized = "true";
    document.body.classList.remove("bilingual-reader-page");

    let layout = content.querySelector(":scope > .bilingual-reader-layout");
    let englishPane;
    let frame;
    let syncing = false;

    function ensureLayout() {
      if (layout) return;
      layout = document.createElement("div");
      layout.className = "bilingual-reader-layout";
      const chinesePane = document.createElement("section");
      chinesePane.className = "bilingual-reader-pane bilingual-reader-pane--chinese";
      const nav = toggle.closest("nav");
      [...content.childNodes].forEach((node) => {
        if (node !== nav) chinesePane.appendChild(node);
      });
      englishPane = document.createElement("section");
      englishPane.className = "bilingual-reader-pane bilingual-reader-pane--english";
      englishPane.lang = "en";
      englishPane.innerHTML = '<h2 class="bilingual-reader-heading">English original</h2>';
      layout.append(chinesePane, englishPane);
      content.appendChild(layout);
    }

    function syncFromPage() {
      if (
        syncing ||
        !document.body.classList.contains("bilingual-reader-page") ||
        !frame?.contentDocument
      ) return;
      const target = frame.contentDocument.scrollingElement;
      const page = document.scrollingElement;
      if (!target || !page) return;
      const pageRange = page.scrollHeight - page.clientHeight;
      const ratio = pageRange > 0 ? page.scrollTop / pageRange : 0;
      syncing = true;
      target.scrollTop = ratio * Math.max(0, target.scrollHeight - target.clientHeight);
      requestAnimationFrame(() => { syncing = false; });
    }

    function syncFromEnglish() {
      if (
        syncing ||
        !document.body.classList.contains("bilingual-reader-page") ||
        !frame?.contentDocument
      ) return;
      const source = frame.contentDocument.scrollingElement;
      const page = document.scrollingElement;
      if (!source || !page) return;
      const sourceRange = source.scrollHeight - source.clientHeight;
      const ratio = sourceRange > 0 ? source.scrollTop / sourceRange : 0;
      syncing = true;
      page.scrollTop = ratio * Math.max(0, page.scrollHeight - page.clientHeight);
      requestAnimationFrame(() => { syncing = false; });
    }

    function bindFrame() {
      const scroller = frame?.contentDocument?.scrollingElement;
      if (!scroller || scroller.dataset.bilingualBound) return;
      scroller.dataset.bilingualBound = "true";
      scroller.addEventListener("scroll", syncFromEnglish, { passive: true });
      syncFromPage();
    }

    toggle.addEventListener("click", () => {
      const opening = !document.body.classList.contains("bilingual-reader-page");
      ensureLayout();
      document.body.classList.toggle("bilingual-reader-page", opening);
      toggle.setAttribute("aria-expanded", String(opening));
      toggle.textContent = opening ? "关闭中英对照" : "中英对照阅读";
      if (opening && !frame) {
        frame = document.createElement("iframe");
        frame.title = "English original";
        frame.loading = "eager";
        frame.src = toggle.dataset.englishUrl;
        frame.addEventListener("load", bindFrame);
        englishPane.appendChild(frame);
      } else if (opening) {
        bindFrame();
      }
    }, { signal });
    window.addEventListener("scroll", syncFromPage, { passive: true, signal });
  }

  documentReady(embedded ? initializeEmbed : () => {
    initializeBilingualReader();
  });
})();
