import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

export function createViewer(onSelect) {
  const $ = id => document.getElementById(id);
  const view = $("view");
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.localClippingEnabled = true;
  view.append(renderer.domElement);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(50, 1, 0.05, 2000);
  camera.position.set(6, 8, 6);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xffffff, 0x59606e, 1.6));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(3, 5, 2); scene.add(sun);
  const clipPlane = new THREE.Plane(new THREE.Vector3(0, -1, 0), 1000);
  const markers = new THREE.Group(); scene.add(markers);
  let model = null, bounds = null, generation = 0;
  const toView = (p) => new THREE.Vector3(p[0], p[2], -p[1]);

  function resize() { const w = view.clientWidth, h = view.clientHeight; renderer.setSize(w, h); camera.aspect = w / Math.max(h, 1); camera.updateProjectionMatrix(); }
  new ResizeObserver(resize).observe(view); resize();
  function bg() { renderer.setClearColor(getComputedStyle(document.documentElement).getPropertyValue("--view").trim() || "#e9ecf1"); }
  bg(); matchMedia("(prefers-color-scheme: dark)").addEventListener("change", bg);
  (function loop() { requestAnimationFrame(loop); controls.update(); const t = performance.now() / 400; for (const m of markers.children) if (m.userData.sel) m.children[0].material.opacity = 0.35 + 0.25 * Math.sin(t); renderer.render(scene, camera); })();

  function release(group) {
    if (!group) return;
    const resources = new Set();
    group.traverse(object => {
      if (object.geometry) resources.add(object.geometry);
      for (const material of [].concat(object.material || [])) {
        resources.add(material);
        for (const value of Object.values(material)) if (value?.isTexture) resources.add(value);
      }
    });
    for (const resource of resources) resource.dispose();
  }
  function clear() {
    ++generation;
    if (model) { scene.remove(model); release(model); }
    release(markers);
    markers.clear();
    model = bounds = null;
  }
  async function load(url, ceiling) {
    clear();
    const token = generation;
    const g = await new GLTFLoader().loadAsync(url);
    if (token !== generation) { release(g.scene); return false; }
    model = g.scene;
    model.traverse(o => {
      if (!o.isMesh) return;
      for (const material of [].concat(o.material)) {
        material.side = THREE.DoubleSide;
        material.clippingPlanes = [clipPlane];
      }
    });
    scene.add(model);
    bounds = new THREE.Box3().setFromObject(model);
    $("clip").value = ceiling ? 55 : 100;
    setClip();
    fit(true);
    return true;
  }
  function setClip() {
    if (!bounds) return;
    const h = THREE.MathUtils.lerp(bounds.min.y, bounds.max.y, Number($("clip").value) / 100);
    clipPlane.constant = $("clip").value >= 100 ? bounds.max.y + 1 : h;
    $("clipV").textContent = $("clip").value >= 100 ? "안 자름" : `${h.toFixed(1)} m 위`;
  }
  $("clip").oninput = setClip;
  function fit(top) {
    if (!bounds) return;
    const c = bounds.getCenter(new THREE.Vector3()), s = bounds.getSize(new THREE.Vector3());
    const halfAngle = Math.atan(Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * Math.min(1, camera.aspect));
    const distance = Math.max(0.5, s.length() / 2) * 1.08 / Math.sin(halfAngle);
    const direction = top ? new THREE.Vector3(0, 1, 0.0001) : new THREE.Vector3(1, 1.3, 1);
    controls.target.copy(c);
    camera.position.copy(c).addScaledVector(direction.normalize(), distance);
    camera.far = Math.max(2000, distance * 8);
    camera.updateProjectionMatrix();
    controls.update();
  }
  $("fit").onclick = () => fit(false);
  $("topv").onclick = () => fit(true);

  function drawMarkers(defects, selected) {
    release(markers);
    markers.clear();
    if (!model) return;
    for (const d of defects) {
      const col = d.severity === "fail" ? 0xe5323e : 0xe39a1c, sel = selected === d.id, r = d.radius_m;
      const g = new THREE.Group(); g.userData = { id: d.id, sel };
      const disc = new THREE.Mesh(new THREE.CylinderGeometry(r, r, sel ? 0.5 : 0.06, 40), new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: sel ? 0.5 : 0.15, depthTest: false, depthWrite: false }));
      disc.position.y = sel ? 0.25 : 0.03;
      const ring = new THREE.Mesh(new THREE.TorusGeometry(r, sel ? 0.05 : 0.025, 8, 48), new THREE.MeshBasicMaterial({ color: col, depthTest: false, depthWrite: false }));
      ring.rotation.x = Math.PI / 2; ring.position.y = 0.04;
      const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, sel ? 2.2 : 1.0, 8), new THREE.MeshBasicMaterial({ color: col, depthTest: false, depthWrite: false }));
      pin.position.y = (sel ? 2.2 : 1.0) / 2;
      g.add(disc, ring, pin); g.renderOrder = 10;
      g.position.copy(toView(d.center_scene));
      markers.add(g);
    }
  }
  function focusDefect(d) {
    const p = toView(d.center_scene), r = Math.max(d.radius_m, 1.5);
    controls.target.copy(p);
    camera.position.set(p.x + r * 1.6, p.y + r * 2.6, p.z + r * 1.6);
  }


  const raycaster = new THREE.Raycaster();
  let press;
  const canvas = renderer.domElement;
  canvas.addEventListener("pointerdown", event => {
    press = event.isPrimary && event.button === 0 ? [event.clientX, event.clientY] : null;
  });
  canvas.addEventListener("pointercancel", () => { press = null; });
  canvas.addEventListener("pointerup", event => {
    const start = press;
    press = null;
    if (!start || !event.isPrimary || Math.hypot(event.clientX - start[0], event.clientY - start[1]) > 5) return;
    const rect = canvas.getBoundingClientRect();
    const mouse = new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1,
      1 - (event.clientY - rect.top) / rect.height * 2);
    raycaster.setFromCamera(mouse, camera);
    const hits = raycaster.intersectObjects(markers.children, true);
    const groups = [...new Set(hits.map(hit => hit.object.parent))];
    if (!groups.length) return;
    const active = groups.find(group => group.userData.sel);
    if (!active) {
      onSelect(groups[0].userData.id);
      return;
    }
    // 선택 표식은 더 높다. 교차 거리 대신 목록 순서로 겹친 표식을 순회한다.
    const ordered = markers.children.filter(group => groups.includes(group));
    onSelect(ordered[(ordered.indexOf(active) + 1) % ordered.length].userData.id);
  });

  return { clear, load, drawMarkers, focusDefect };
}
