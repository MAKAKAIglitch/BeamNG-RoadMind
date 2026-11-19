import { lua } from "@/bridge"

const BUTTONS = [
  {
    id: "roadmind-settings",
    tabId: "mods",
    label: "RoadMind",
    icon: "cogOutline",
    componentName: "/ui/ui-vue/mods/roadmind/Settings.vue",
  },
  {
    id: "roadmind-adas",
    tabId: "mods",
    label: "RoadMind ADAS",
    icon: "cogOutline",
    componentName: "/ui/ui-vue/mods/roadmind/ADAS.vue",
  },
  {
    id: "roadmind-perception",
    tabId: "mods",
    label: "RoadMind Perception",
    icon: "cogOutline",
    componentName: "/ui/ui-vue/mods/roadmind/Perception.vue",
  },
]

export async function onLoad() {
  for (const button of BUTTONS) {
    try {
      const result = await lua.extensions.ui_pause_actions.registerModButton(button)
      if (!result || result.success !== true) {
        throw new Error("registerModButton failed: " + (result && result.reason || "unknown error"))
      }
    } catch (error) {
      console.error("RoadMind: failed to register " + button.id, error)
    }
  }
}

export async function onUnload() {
  for (const button of BUTTONS) {
    try {
      const result = await lua.extensions.ui_pause_actions.unregisterModButton(button.id)
      if (result && result.success === false && result.reason !== "not_found") {
        throw new Error("unregisterModButton failed: " + result.reason)
      }
    } catch (error) {
      console.error("RoadMind: failed to unregister " + button.id, error)
    }
  }
}
