(() => {
  const embedded = new URLSearchParams(window.location.search).has("embed");
  const desktopReaderQuery = window.matchMedia("(min-width: 64.01em)");
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
      document.documentElement.classList.remove("bilingual-reader-active");
      return;
    }
    if (toggle.dataset.bilingualInitialized === "true") return;

    bilingualController?.abort();
    bilingualController = new AbortController();
    const { signal } = bilingualController;
    toggle.dataset.bilingualInitialized = "true";
    document.body.classList.remove("bilingual-reader-page");
    document.documentElement.classList.remove("bilingual-reader-active");

    let layout = content.querySelector(":scope > .bilingual-reader-layout");
    let chinesePane = layout?.querySelector(".bilingual-reader-pane--chinese");
    let englishPane = layout?.querySelector(".bilingual-reader-pane--english");
    let frame = englishPane?.querySelector("iframe");
    let frameController;
    let syncFrame;
    let syncMonitor;
    let syncingPane;
    let lastChineseTop = 0;
    let lastEnglishTop = 0;
    let savedPageTop = 0;

    const scrollRange = (element) => Math.max(0, element.scrollHeight - element.clientHeight);

    function pageProgress() {
      const page = document.scrollingElement;
      const range = page ? scrollRange(page) : 0;
      return range > 0 ? page.scrollTop / range : 0;
    }

    function ensureLayout() {
      if (layout) return;
      layout = document.createElement("div");
      layout.className = "bilingual-reader-layout";
      chinesePane = document.createElement("section");
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

    function readerIsActive() {
      return desktopReaderQuery.matches &&
        document.body.classList.contains("bilingual-reader-page");
    }

    function syncScroll(source) {
      if (!readerIsActive() || !frame?.contentDocument || !chinesePane) return;
      const englishScroller = frame.contentDocument.scrollingElement;
      const from = source === "zh" ? chinesePane : englishScroller;
      const to = source === "zh" ? englishScroller : chinesePane;
      const fromRange = scrollRange(from);
      const ratio = fromRange > 0 ? from.scrollTop / fromRange : 0;
      syncingPane = source === "zh" ? "en" : "zh";
      to.scrollTop = ratio * scrollRange(to);
      lastChineseTop = chinesePane.scrollTop;
      lastEnglishTop = englishScroller.scrollTop;
      requestAnimationFrame(() => {
        if (syncingPane === (source === "zh" ? "en" : "zh")) syncingPane = undefined;
      });
    }

    function syncFromChinese() {
      if (syncingPane === "zh") return;
      cancelAnimationFrame(syncFrame);
      syncFrame = requestAnimationFrame(() => syncScroll("zh"));
    }

    function startScrollMonitor() {
      clearInterval(syncMonitor);
      syncMonitor = window.setInterval(() => {
        if (!readerIsActive() || !chinesePane) return;
        const englishScroller = frame?.contentDocument?.scrollingElement;
        if (!englishScroller) return;
        const chineseChanged = Math.abs(chinesePane.scrollTop - lastChineseTop) > 0.5;
        const englishChanged = Math.abs(englishScroller.scrollTop - lastEnglishTop) > 0.5;
        if (englishChanged && !chineseChanged && syncingPane !== "en") {
          syncScroll("en");
        } else if (chineseChanged && !englishChanged && syncingPane !== "zh") {
          syncScroll("zh");
        }
        lastChineseTop = chinesePane.scrollTop;
        lastEnglishTop = englishScroller.scrollTop;
      }, 40);
    }

    function bindFrame() {
      const frameDocument = frame?.contentDocument;
      const scroller = frameDocument?.scrollingElement;
      if (!scroller) return;
      frameController?.abort();
      frameController = new AbortController();
      frame.contentWindow.addEventListener("scroll", () => {
        if (syncingPane !== "en") {
          syncScroll("en");
        }
      }, { passive: true, signal: frameController.signal });
      syncScroll("zh");
      startScrollMonitor();
    }

    function updateReaderHeight() {
      if (!readerIsActive() || !layout) return;
      const top = Math.max(0, layout.getBoundingClientRect().top);
      const available = Math.max(320, window.innerHeight - top - 12);
      layout.style.setProperty("--bilingual-reader-height", `${available}px`);
    }

    function openReader(initialProgress) {
      ensureLayout();
      savedPageTop = document.scrollingElement?.scrollTop || 0;
      if (desktopReaderQuery.matches) window.scrollTo(0, 0);
      document.body.classList.add("bilingual-reader-page");
      document.documentElement.classList.toggle(
        "bilingual-reader-active",
        desktopReaderQuery.matches
      );
      toggle.setAttribute("aria-expanded", "true");
      toggle.textContent = "关闭中英对照";

      if (!frame) {
        frame = document.createElement("iframe");
        frame.title = "English original";
        frame.loading = "eager";
        frame.src = toggle.dataset.englishUrl;
        frame.addEventListener("load", bindFrame, { signal });
        englishPane.appendChild(frame);
      }

      requestAnimationFrame(() => {
        updateReaderHeight();
        startScrollMonitor();
        if (desktopReaderQuery.matches && chinesePane) {
          chinesePane.scrollTop = initialProgress * scrollRange(chinesePane);
          bindFrame();
        }
      });
    }

    function closeReader() {
      const progress = chinesePane && scrollRange(chinesePane) > 0
        ? chinesePane.scrollTop / scrollRange(chinesePane)
        : pageProgress();
      cancelAnimationFrame(syncFrame);
      clearInterval(syncMonitor);
      document.body.classList.remove("bilingual-reader-page");
      document.documentElement.classList.remove("bilingual-reader-active");
      toggle.setAttribute("aria-expanded", "false");
      toggle.textContent = "中英对照阅读";
      requestAnimationFrame(() => {
        const page = document.scrollingElement;
        if (!page) return;
        const restored = desktopReaderQuery.matches
          ? progress * scrollRange(page)
          : savedPageTop;
        window.scrollTo(0, restored);
      });
    }

    toggle.addEventListener("click", () => {
      const opening = !document.body.classList.contains("bilingual-reader-page");
      if (opening) {
        const initialProgress = pageProgress();
        ensureLayout();
        if (!chinesePane.dataset.bilingualBound) {
          chinesePane.dataset.bilingualBound = "true";
          chinesePane.addEventListener("scroll", syncFromChinese, { passive: true, signal });
        }
        openReader(initialProgress);
      } else {
        closeReader();
      }
    }, { signal });

    window.addEventListener("resize", updateReaderHeight, { passive: true, signal });
    desktopReaderQuery.addEventListener("change", () => {
      if (!document.body.classList.contains("bilingual-reader-page")) return;
      if (desktopReaderQuery.matches) {
        document.documentElement.classList.add("bilingual-reader-active");
        window.scrollTo(0, 0);
        requestAnimationFrame(() => {
          updateReaderHeight();
          startScrollMonitor();
        });
      } else {
        clearInterval(syncMonitor);
        layout?.style.removeProperty("--bilingual-reader-height");
        document.documentElement.classList.remove("bilingual-reader-active");
      }
    }, { signal });
    signal.addEventListener("abort", () => {
      frameController?.abort();
      cancelAnimationFrame(syncFrame);
      clearInterval(syncMonitor);
      document.documentElement.classList.remove("bilingual-reader-active");
    }, { once: true });
  }

  documentReady(embedded ? initializeEmbed : initializeBilingualReader);
})();
