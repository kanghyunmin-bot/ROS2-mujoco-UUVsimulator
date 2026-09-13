// Shared modal lifecycle. Closing a window never sends a robot/recorder command.
(() => {
  const stack = [];
  const focusable = root => [...root.querySelectorAll('button:not(:disabled),input:not(:disabled),select:not(:disabled),textarea:not(:disabled),summary,[tabindex="0"]')].filter(el => el.getClientRects().length);
  function sync() {
    const top = stack.at(-1)?.element;
    document.body.classList.toggle('dialog-open', Boolean(top));
    for (const root of document.querySelectorAll('body > header,body > main,body > .dialog')) {
      root.inert = Boolean(top && root !== top);
    }
    stack.forEach(({element}, index) => { element.style.zIndex = String(20 + index); });
  }
  function open(id, opener = document.activeElement) {
    const element = document.getElementById(id);
    if (!element || stack.some(item => item.element === element)) return;
    const heading = element.querySelector('h2');
    if (heading) {
      if (!heading.id) heading.id = id + 'Title';
      element.setAttribute('aria-labelledby', heading.id);
    }
    element.setAttribute('role', 'dialog');
    element.setAttribute('aria-modal', 'true');
    element.tabIndex = -1;
    stack.push({element, previous: opener});
    element.classList.remove('hidden');
    sync();
    (focusable(element)[0] || element).focus();
  }
  function close(id) {
    const item = stack.at(-1);
    if (!item || item.element.id !== id) return;
    stack.pop();
    item.element.classList.add('hidden');
    item.element.removeAttribute('aria-modal');
    sync();
    item.element.dispatchEvent(new CustomEvent('station-window-closed'));
    if (item.previous?.isConnected && !item.previous.closest('[inert]')) item.previous.focus();
  }
  document.addEventListener('keydown', event => {
    const item = stack.at(-1);
    if (!item) return;
    if (event.key === 'Escape') {
      event.preventDefault(); event.stopImmediatePropagation(); close(item.element.id);
    } else if (event.key === 'Tab') {
      const items = focusable(item.element);
      const first = items[0] || item.element, last = items.at(-1) || first;
      if (event.shiftKey && (document.activeElement === first || document.activeElement === item.element)) { event.preventDefault(); last.focus(); }
      else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first.focus(); }
    }
  }, true);
  // Backdrop clicks deliberately keep edited values and the window open.
  window.StationWindows = {open, close};
})();
