// MAME-JavaScript function mappings
var JSMAME = JSMAME || {};
JSMAME.get_machine = function () { return Module.cwrap('_ZN15running_machine30emscripten_get_running_machineEv', 'number').apply(null, arguments); };
JSMAME.get_ui = function () { return Module.cwrap('_ZN15running_machine17emscripten_get_uiEv', 'number').apply(null, arguments); };
JSMAME.get_sound = function () { return Module.cwrap('_ZN15running_machine20emscripten_get_soundEv', 'number').apply(null, arguments); };
JSMAME.set_bgfx_chain = function () { return Module.cwrap('_ZN15running_machine25emscripten_set_bgfx_chainEPKc', 'number', ['string']).apply(null, arguments); };
JSMAME.resize_window = function () { return Module.cwrap('_ZN15running_machine24emscripten_resize_windowEii', '', ['number', 'number']).apply(null, arguments); };
JSMAME.set_keepaspect = function () { return Module.cwrap('_ZN15running_machine25emscripten_set_keepaspectEi', '', ['number']).apply(null, arguments); };
JSMAME.set_fastforward = function () { return Module.cwrap('_ZN15running_machine26emscripten_set_fastforwardEi', '', ['number']).apply(null, arguments); };
JSMAME.cassette_toggle = function () { return Module.cwrap('_ZN15running_machine26emscripten_cassette_toggleEv', 'number', []).apply(null, arguments); };
JSMAME.ui_set_show_fps = function () { return Module.cwrap('_ZN15mame_ui_manager12set_show_fpsEb', '', ['number', 'number']).apply(null, arguments); };
JSMAME.ui_get_show_fps = function () { return Module.cwrap('_ZNK15mame_ui_manager8show_fpsEv', 'number', ['number']).apply(null, arguments); };
JSMAME.sound_manager_mute = function () { return Module.cwrap('_ZN13sound_manager4muteEbh', '', ['number', 'number', 'number']).apply(null, arguments); };
JSMAME.sdl_pauseaudio = function () { return Module.cwrap('SDL_PauseAudio', '', ['number']).apply(null, arguments); };
JSMAME.sdl_sendkeyboardkey = function () { return Module.cwrap('SDL_SendKeyboardKey', '', ['number', 'number']).apply(null, arguments); };

JSMAME.soft_reset = function () { return Module.cwrap('_ZN15running_machine21emscripten_soft_resetEv', null).apply(null, arguments); };
JSMAME.hard_reset = function () { return Module.cwrap('_ZN15running_machine21emscripten_hard_resetEv', null).apply(null, arguments); };
JSMAME.exit = function () { return Module.cwrap('_ZN15running_machine15emscripten_exitEv', null, []).apply(null, arguments); };
JSMAME.save = function () { return Module.cwrap('_ZN15running_machine15emscripten_saveEPKc', null, ['string']).apply(null, arguments); };
JSMAME.load = function () { return Module.cwrap('_ZN15running_machine15emscripten_loadEPKc', null, ['string']).apply(null, arguments); };

var JSMESS = JSMAME;
// mame.js ships wrapped in an IIFE, so publish the bridge explicitly;
// the loader and theme call window.JSMAME.
globalThis.JSMAME = JSMAME;
globalThis.JSMESS = JSMAME;

// Headless CLI support (node; see mame-node.mjs in the emame repo): the
// launcher sets JSMAME.preload = { files: [[path, Uint8Array], ...],
// dirs: [path, ...] } after this module evaluates but before main() runs,
// making host paths (ROM zips, autoboot scripts, writable state dirs)
// visible inside the emscripten filesystem.
JSMAME.preload = null;
Module.preRun = Module.preRun || [];
Module.preRun.push(function () {
	var p = JSMAME.preload;
	if (!p)
		return;
	(p.dirs || []).forEach(function (d) { FS.mkdirTree(d); });
	(p.files || []).forEach(function (f) {
		FS.mkdirTree(f[0].substring(0, f[0].lastIndexOf("/")) || "/");
		FS.writeFile(f[0], f[1]);
	});
});
