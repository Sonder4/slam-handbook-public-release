(() => {
  const desktopQuery = window.matchMedia("(min-width: 76.25em)");
  const controls = [
    {
      buttonClass: "reader-sidebar-toggle--primary",
      bodyClass: "reader-primary-collapsed",
      label: "收起章节导航",
      expandedLabel: "展开章节导航",
      expandedSymbol: "<",
      collapsedSymbol: ">"
    },
    {
      buttonClass: "reader-sidebar-toggle--secondary",
      bodyClass: "reader-secondary-collapsed",
      label: "收起页内目录",
      expandedLabel: "展开页内目录",
      expandedSymbol: ">",
      collapsedSymbol: "<"
    }
  ];

  function restoreControl(control) {
    const stored = localStorage.getItem(control.bodyClass) === "true";
    document.body.classList.toggle(control.bodyClass, stored);
  }

  function syncButton(button, control) {
    const collapsed = document.body.classList.contains(control.bodyClass);
    button.setAttribute("aria-expanded", String(!collapsed));
    button.setAttribute("aria-label", collapsed ? control.expandedLabel : control.label);
    button.textContent = collapsed ? control.collapsedSymbol : control.expandedSymbol;
  }

  function initialize() {
    document.querySelectorAll(".reader-sidebar-toggle").forEach((button) => button.remove());
    if (!desktopQuery.matches) {
      controls.forEach((control) => document.body.classList.remove(control.bodyClass));
      return;
    }
    controls.forEach((control) => {
      restoreControl(control);
      const button = document.createElement("button");
      button.type = "button";
      button.className = "reader-sidebar-toggle " + control.buttonClass;
      button.addEventListener("click", () => {
        document.body.classList.toggle(control.bodyClass);
        localStorage.setItem(
          control.bodyClass,
          String(document.body.classList.contains(control.bodyClass))
        );
        syncButton(button, control);
      });
      syncButton(button, control);
      document.body.appendChild(button);
    });
  }

  if (typeof document$ !== "undefined") {
    document$.subscribe(initialize);
  } else {
    document.addEventListener("DOMContentLoaded", initialize);
  }
  desktopQuery.addEventListener("change", initialize);
})();
