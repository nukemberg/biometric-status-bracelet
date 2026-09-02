// Network-first app shell cache. Web Bluetooth needs the freshest decode
// logic against whatever protocol version the device speaks (see index.html's
// header comment), so this never serves a cached response ahead of a live
// network fetch -- cache is a fallback for when there's no network at all
// (e.g. offline at a festival), not a performance optimization.
const CACHE = "biomonitor-shell-v1";
const SHELL = ["./", "index.html", "manifest.json", "icon.svg"];

self.addEventListener("install", (event) => {
  event.waitUntil(caches.open(CACHE).then((cache) => cache.addAll(SHELL)));
  self.skipWaiting();
});

self.addEventListener("activate", (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k)))
    )
  );
  self.clients.claim();
});

self.addEventListener("fetch", (event) => {
  if (event.request.method !== "GET") return;
  event.respondWith(
    fetch(event.request)
      .then((response) => {
        const copy = response.clone();
        caches.open(CACHE).then((cache) => cache.put(event.request, copy));
        return response;
      })
      .catch(() => caches.match(event.request))
  );
});
