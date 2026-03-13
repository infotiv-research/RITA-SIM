import omni.kit.app
import omni.ext

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.anim.graph.bundle", True)
ext_manager.set_extension_enabled_immediate("omni.anim.graph.core", True)
ext_manager.set_extension_enabled_immediate("omni.anim.graph.ui", True)
ext_manager.set_extension_enabled_immediate("omni.anim.skelJoint", True)
ext_manager.set_extension_enabled_immediate("isaacsim.asset.exporter.urdf", True)

print("Extensions loaded.")
