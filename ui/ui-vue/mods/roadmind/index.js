import { lua } from "@/bridge"

const BUTTON_ID = "roadmind-settings"
const COMPONENT = "/ui/ui-vue/mods/roadmind/Settings.vue"

export async function onLoad() {
  try {
    const result = await lua.extensions.ui_pause_actions.registerModButton({
      id: BUTTON_ID,
      tabId: "mods",
      label: "RoadMind",
      icon: "cogOutline",
      componentName: COMPONENT,
    })
    if (!result || result.success !== true) {
      throw new Error("registerModButton failed: " + (result && result.reason || "unknown error"))
    }
  } catch (error) {
    console.error("RoadMind: failed to register pause-menu settings", error)
  }
}

export async function onUnload() {
  try {
    const result = await lua.extensions.ui_pause_actions.unregisterModButton(BUTTON_ID)
    if (result && result.success === false && result.reason !== "not_found") {
      throw new Error("unregisterModButton failed: " + result.reason)
    }
  } catch (error) {
    console.error("RoadMind: failed to unregister pause-menu settings", error)
  }
}
