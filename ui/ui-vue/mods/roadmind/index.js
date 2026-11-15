import { lua } from "@/bridge"

const BUTTON_ID = "roadmind-settings"
const COMPONENT = "/ui/ui-vue/mods/roadmind/Settings.vue"

export async function onLoad() {
  try {
    await lua.extensions.ui_pause_actions.registerModButton({
      id: BUTTON_ID,
      tabId: "mods",
      label: "RoadMind",
      icon: "cogOutline",
      componentName: COMPONENT,
    })
  } catch (error) {
    console.error("RoadMind: failed to register pause-menu settings", error)
  }
}

export async function onUnload() {
  try {
    await lua.extensions.ui_pause_actions.unregisterModButton(BUTTON_ID)
  } catch (error) {
    console.error("RoadMind: failed to unregister pause-menu settings", error)
  }
}
