#include <rviz/display.h>
#include <rviz/display_context.h>
#include <rviz/properties/status_property.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>
#include <Overlay/OgreTextAreaOverlayElement.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <sstream>

namespace visualizer {
class TrafficSignalDisplay : public rviz::Display {
 public:
  ~TrafficSignalDisplay() override { cleanup(); }

  void reset() override {
    has_message_ = false;
    guarded([this] {
      if (overlay_) overlay_->hide();
      if (context_) context_->queueRender();
    });
  }

  void update(float, float) override {
    if (has_message_ && std::chrono::steady_clock::now() >= expires_at_) reset();
  }

 protected:
  void onInitialize() override {
    ready_ = guarded([this] {
      std::ostringstream name;
      name << "TrafficSignalOverlay_" << this;
      auto& manager = Ogre::OverlayManager::getSingleton();
      overlay_ = manager.create(name.str());
      panel_ = static_cast<Ogre::PanelOverlayElement*>(
          manager.createOverlayElement("Panel", name.str() + "_Panel"));
      text_ = static_cast<Ogre::TextAreaOverlayElement*>(
          manager.createOverlayElement("TextArea", name.str() + "_Text"));
      panel_->setMetricsMode(Ogre::GMM_RELATIVE);
      panel_->setDimensions(1, 1);
      panel_->setTransparent(true);
      text_->setMetricsMode(Ogre::GMM_PIXELS);
      text_->setHorizontalAlignment(Ogre::GHA_RIGHT);
      text_->setVerticalAlignment(Ogre::GVA_TOP);
      text_->setPosition(-16, 16);
      text_->setDimensions(600, 32);
      text_->setAlignment(Ogre::TextAreaOverlayElement::Right);
      text_->setFontName("Liberation Sans");
      text_->setCharHeight(24);
      panel_->addChild(text_);
      overlay_->add2D(panel_);
      overlay_->setZOrder(500);
    });
    if (!ready_) cleanup();
  }

  void onEnable() override {
    if (!ready_) return;
    guarded([this] {
      subscriber_ = update_nh_.subscribe("/visualizer/traffic_light", 1,
                                       &TrafficSignalDisplay::receiveMarker, this);
    });
  }

  void onDisable() override {
    guarded([this] { subscriber_.shutdown(); });
    reset();
  }

 private:
  template <class Function>
  bool guarded(Function function) noexcept {
    try {
      function();
      return true;
    } catch (const std::exception& error) {
      ROS_ERROR("Traffic signal overlay: %s", error.what());
    } catch (...) {
      ROS_ERROR("Traffic signal overlay: unknown display error");
    }
    return false;
  }

  void receiveMarker(const visualization_msgs::Marker::ConstPtr& marker) {
    if (!guarded([this, &marker] {
          if (marker->action == visualization_msgs::Marker::DELETE ||
              marker->action == visualization_msgs::Marker::DELETEALL) {
            reset();
            return;
          }
          if (marker->action != visualization_msgs::Marker::ADD ||
              marker->type != visualization_msgs::Marker::TEXT_VIEW_FACING) return;
          if (marker->text.empty()) {
            reset();
            return;
          }
          const auto& color = marker->color;
          const float components[] = {color.r, color.g, color.b, color.a};
          for (float component : components) {
            if (!std::isfinite(component) || component < 0 || component > 1) {
              reset();
              setStatus(rviz::StatusProperty::Warn, "Overlay", "Invalid text color");
              return;
            }
          }
          // The Core emits printable ASCII; reject malformed text before Ogre renders it.
          if (marker->text.size() > 256 ||
              std::any_of(marker->text.begin(), marker->text.end(),
                          [](unsigned char character) { return character < 32 || character > 126; })) {
            reset();
            setStatus(rviz::StatusProperty::Warn, "Overlay", "Invalid traffic signal text");
            return;
          }
          text_->setCaption(marker->text);
          text_->setColour(Ogre::ColourValue(color.r, color.g, color.b, color.a));
          expires_at_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
          has_message_ = true;
          overlay_->show();
          setStatus(rviz::StatusProperty::Ok, "Overlay", "Traffic signal received");
          context_->queueRender();
        })) reset();
  }

  void cleanup() noexcept {
    ready_ = false;
    has_message_ = false;
    guarded([this] { subscriber_.shutdown(); });
    auto* manager = Ogre::OverlayManager::getSingletonPtr();
    if (!manager) return;
    guarded([this] {
      if (overlay_) overlay_->hide();
      if (overlay_ && panel_) overlay_->remove2D(panel_);
      if (panel_ && text_ && text_->getParent() == panel_) panel_->removeChild(text_->getName());
    });
    auto* text = text_;
    text_ = nullptr;
    guarded([manager, text] {
      if (text) manager->destroyOverlayElement(text);
    });
    auto* panel = panel_;
    panel_ = nullptr;
    guarded([manager, panel] {
      if (panel) manager->destroyOverlayElement(panel);
    });
    auto* overlay = overlay_;
    overlay_ = nullptr;
    guarded([manager, overlay] {
      if (overlay) manager->destroy(overlay);
    });
  }

  ros::Subscriber subscriber_;
  Ogre::Overlay* overlay_ = nullptr;
  Ogre::PanelOverlayElement* panel_ = nullptr;
  Ogre::TextAreaOverlayElement* text_ = nullptr;
  std::chrono::steady_clock::time_point expires_at_;
  bool ready_ = false;
  bool has_message_ = false;
};
}  // namespace visualizer

PLUGINLIB_EXPORT_CLASS(visualizer::TrafficSignalDisplay, rviz::Display)
