(() => {
  const embedded = new URLSearchParams(window.location.search).has("embed");

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

  function initializeParallelReader() {
    const reader = document.querySelector("[data-parallel-reader]");
    if (!reader) return;

    document.body.classList.add("parallel-reader-page");
    const frames = [...reader.querySelectorAll("iframe")];
    const boundFrames = new WeakSet();
    let syncing = false;

    function synchronize(source) {
      if (syncing) return;
      const sourceScroller = source.contentDocument?.scrollingElement;
      if (!sourceScroller) return;
      const sourceRange = sourceScroller.scrollHeight - sourceScroller.clientHeight;
      const position = sourceRange > 0 ? sourceScroller.scrollTop / sourceRange : 0;

      syncing = true;
      frames.forEach((frame) => {
        if (frame === source) return;
        const targetScroller = frame.contentDocument?.scrollingElement;
        if (!targetScroller) return;
        targetScroller.scrollTop = position * (targetScroller.scrollHeight - targetScroller.clientHeight);
      });
      requestAnimationFrame(() => {
        syncing = false;
      });
    }

    function bindFrame(frame) {
      if (boundFrames.has(frame)) return;
      const scroller = frame.contentDocument?.scrollingElement;
      if (!scroller) return;
      boundFrames.add(frame);
      scroller.addEventListener("scroll", () => synchronize(frame), { passive: true });
    }

    frames.forEach((frame) => {
      frame.addEventListener("load", () => bindFrame(frame));
      if (frame.contentDocument?.readyState === "complete") bindFrame(frame);
    });
  }

  documentReady(embedded ? initializeEmbed : initializeParallelReader);
})();
