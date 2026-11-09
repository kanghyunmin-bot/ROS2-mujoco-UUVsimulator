// The station defaults to English and remembers the operator's choice in this browser.
(() => {
  const key = "uuv.language";
  const fallback = new URL(window.location.href).searchParams.get("lang");
  let language = fallback === "ko" ? "ko" : "en";
  try {
    if (fallback === null && localStorage.getItem(key) === "ko") language = "ko";
  } catch { /* storage may be unavailable */ }
  const text = (english, korean) => language === "ko" ? korean : english;
  window.StationLanguage = { text };
  document.documentElement.lang = language;

  const attributes = ["aria-label", "alt", "placeholder", "title"];
  if (language === "ko") {
    document.querySelectorAll("[data-ko]").forEach(element => {
      element.textContent = element.dataset.ko;
    });
    for (const attribute of attributes) {
      const translated = `data-ko-${attribute}`;
      document.querySelectorAll(`[${translated}]`).forEach(element => {
        element.setAttribute(attribute, element.getAttribute(translated));
      });
    }
  }

  const select = document.getElementById("stationLanguage");
  select.value = language;
  select.addEventListener("change", () => {
    const url = new URL(window.location.href);
    try {
      localStorage.setItem(key, select.value);
      if (!url.searchParams.has("lang")) {
        window.location.reload();
        return;
      }
      url.searchParams.delete("lang");
    } catch {
      url.searchParams.set("lang", select.value);
    }
    window.location.replace(url.href);
  });
})();
