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
    let syncLockUntil = 0;
    let activeScrollSource = "zh";
    let lastChineseTop = 0;
    let lastEnglishTop = 0;
    let savedPageTop = 0;
    let readerToolbar;

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
      layout.append(chinesePane, englishPane);
      content.appendChild(layout);
    }

    function readerIsActive() {
      return desktopReaderQuery.matches &&
        document.body.classList.contains("bilingual-reader-page");
    }

    function anchorPositions(root, scroller) {
      const scrollerTop = scroller === root.scrollingElement
        ? 0
        : scroller.getBoundingClientRect().top;
      const seen = new Set();
      return [...root.querySelectorAll(".equation-anchor[id], .figure-anchor[id]")]
        .filter((element) => {
          if (
            (!element.id.startsWith("eq-") && !element.id.startsWith("fig-")) ||
            seen.has(element.id)
          ) return false;
          seen.add(element.id);
          return true;
        })
        .map((element) => ({
          id: element.id,
          top: element.getBoundingClientRect().top - scrollerTop + scroller.scrollTop
        }))
        .sort((a, b) => a.top - b.top);
    }

    function synchronizedTop(source, target, sourceDocument, targetDocument) {
      const sourceRange = scrollRange(source);
      const targetRange = scrollRange(target);
      if (!sourceRange || !targetRange) return 0;
      const targetAnchors = new Map();
      anchorPositions(targetDocument, target).forEach((anchor) => {
        if (!targetAnchors.has(anchor.id)) targetAnchors.set(anchor.id, anchor.top);
      });
      const rawPairs = anchorPositions(sourceDocument, source)
        .filter((anchor) => targetAnchors.has(anchor.id))
        .map((anchor) => {
          const sourceMargin = parseFloat(
            sourceDocument.defaultView.getComputedStyle(
              sourceDocument.getElementById(anchor.id)
            ).scrollMarginTop
          ) || 0;
          const targetMargin = parseFloat(
            targetDocument.defaultView.getComputedStyle(
              targetDocument.getElementById(anchor.id)
            ).scrollMarginTop
          ) || 0;
          return {
            source: Math.max(0, anchor.top - sourceMargin),
            target: Math.max(0, targetAnchors.get(anchor.id) - targetMargin)
          };
        })
        .filter((pair) => pair.source >= 0 && pair.target >= 0);
      const pairs = rawPairs.filter((pair, index) => {
        if (!index) return true;
        const previous = rawPairs[index - 1];
        return pair.source > previous.source && pair.target > previous.target;
      });
      const points = [];
      [{ source: 0, target: 0 }, ...pairs, {
        source: sourceRange,
        target: targetRange
      }].forEach((point) => {
        const previous = points[points.length - 1];
        if (!previous || (point.source > previous.source && point.target > previous.target)) {
          points.push(point);
        }
      });
      const current = source.scrollTop;
      let upperIndex = points.findIndex((point) => point.source > current);
      if (upperIndex < 0) upperIndex = points.length - 1;
      const lower = points[Math.max(0, upperIndex - 1)];
      const upper = points[upperIndex];
      const span = Math.max(1, upper.source - lower.source);
      const progress = Math.min(1, Math.max(0, (current - lower.source) / span));
      return lower.target + progress * (upper.target - lower.target);
    }

    function syncScroll(source) {
      if (!readerIsActive() || !frame?.contentDocument || !chinesePane) return;
      const englishScroller = frame.contentDocument.scrollingElement;
      const from = source === "zh" ? chinesePane : englishScroller;
      const to = source === "zh" ? englishScroller : chinesePane;
      const fromDocument = source === "zh" ? document : frame.contentDocument;
      const toDocument = source === "zh" ? frame.contentDocument : document;
      syncingPane = source === "zh" ? "en" : "zh";
      syncLockUntil = performance.now() + 160;
      to.scrollTop = synchronizedTop(from, to, fromDocument, toDocument);
      lastChineseTop = chinesePane.scrollTop;
      lastEnglishTop = englishScroller.scrollTop;
      window.setTimeout(() => {
        if (performance.now() >= syncLockUntil) syncingPane = undefined;
      }, 180);
    }

    function syncFromChinese() {
      if (syncingPane === "zh" || activeScrollSource !== "zh") return;
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
        if (
          englishChanged &&
          !chineseChanged &&
          syncingPane !== "en" &&
          performance.now() >= syncLockUntil &&
          activeScrollSource === "en"
        ) {
          syncScroll("en");
        } else if (
          chineseChanged &&
          !englishChanged &&
          syncingPane !== "zh" &&
          performance.now() >= syncLockUntil &&
          activeScrollSource === "zh"
        ) {
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
      frame.contentDocument.addEventListener("wheel", () => {
        activeScrollSource = "en";
      }, { passive: true, signal: frameController.signal });
      frame.contentDocument.addEventListener("pointerdown", () => {
        activeScrollSource = "en";
      }, { passive: true, signal: frameController.signal });
      frame.contentDocument.addEventListener("keydown", () => {
        activeScrollSource = "en";
      }, { signal: frameController.signal });
      frame.contentWindow.addEventListener("scroll", () => {
        if (
          syncingPane !== "en" &&
          performance.now() >= syncLockUntil &&
          activeScrollSource === "en"
        ) {
          syncScroll("en");
        }
      }, { passive: true, signal: frameController.signal });
      syncScroll("zh");
      startScrollMonitor();
    }

    function updateReaderHeight() {
      if (!readerIsActive() || !layout) return;
      layout.style.removeProperty("--bilingual-reader-height");
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
      document.documentElement.requestFullscreen?.().catch(() => {});

      if (!readerToolbar) {
        readerToolbar = document.createElement("header");
        readerToolbar.className = "bilingual-reader-toolbar";
        readerToolbar.innerHTML = [
          '<strong lang="zh-CN">中文译文</strong>',
          '<strong lang="en">English original</strong>',
          '<button type="button" class="bilingual-reader-close" aria-label="关闭中英对照">×</button>'
        ].join("");
        readerToolbar.querySelector("button").addEventListener("click", closeReader, { signal });
        layout.before(readerToolbar);
      }

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
      if (document.fullscreenElement) document.exitFullscreen?.().catch(() => {});
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
          chinesePane.addEventListener("wheel", () => {
            activeScrollSource = "zh";
          }, { passive: true, signal });
          chinesePane.addEventListener("pointerdown", () => {
            activeScrollSource = "zh";
          }, { passive: true, signal });
          chinesePane.addEventListener("keydown", () => {
            activeScrollSource = "zh";
          }, { signal });
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
