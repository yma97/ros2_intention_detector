import rclpy
import pyglet
import re
from rclpy.node import Node

from std_msgs.msg import String
from justhink_world import create_world, load_log, show_world, world
from justhink_world.agent import Human, Robot
from justhink_world.domain.action import PickAction, ClearAction, \
AttemptSubmitAction, ContinueAction, SubmitAction, AgreeAction, DisagreeAction
from justhink_world.visual import WorldWindow

class IntentionSubscriber(Node):

    def __init__(self):
        # Call back to detect user's intention
        super().__init__('intention_subscriber')

        # Create a world.
        self.world = create_world('collaboration-1')
        print("initialized world")
        self.win = WorldWindow(self.world, state_no=None, screen_index=-1)
        print("world window intitialized 2")
        self.instuction_msg = ""

        # Start intention detection
        self.subscription = self.create_subscription(
            String,
            'intention',
            self.listener_callback,
            10)
        print("start recognition")
        self.subscription  # prevent unused variable warning
        print("recognition finish")
        # # Override the updater for next label.
        # def _update_next_label():
        #     pass
        # win._update_next_label = _update_next_label
        # # Change the text e.g. in a callback.
        # win.graphics.next_label.text = self.instruction_msg


    def listener_callback(self, msg):  
        self.get_logger().info('I heard: "%s"' % msg.data)
        # Excute the action in the activity
        # if msg.data == agree:
        #     world.act(AgreeAction(agent=Human))
        # elif msg.data == disagree:
        #     world.act(DisagreeAction(agent=Human))
        
        if not msg.data == "error":

            intention_dic = self.keyword_detection(msg.data)
            intention = ""
            #current_agent = Human  #self.world.agent

            if intention_dic["connect"]:
                intention = "connect"
                #print("User wants to {} ".format(intention))
            elif intention_dic["clearall"]:
                intention = "clear all"
                #print("User wants to {} ".format(intention))
            elif intention_dic["submit"]:
                intention = "submit"
                #print("User wants to {} ".format(intention))
            elif intention_dic["agree"]:
                intention = "agree"
                if AgreeAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(AgreeAction(agent=Human))
                else:
                    self.win.execute_action(AgreeAction(agent=Robot))
                # self.instuction_msg = "User wants to {} ".format(intention)
            elif intention_dic["disagree"]:
                intention = "disagree"
                if DisagreeAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(DisagreeAction(agent=Human))
                else:
                    self.win.execute_action(DisagreeAction(agent=Robot))
                # self.instuction_msg = "User wants to {} ".format(intention)
            else:
                print("Error: intention detection failed.")

            self.instuction_msg = "User wants to {} ".format(intention)
            # self.get_logger().info('User wants to: "%s"' % intention)
            # return intention


   
    def keyword_detection(self,transcript):
        """Detect user's intention when speech recognition succeeds.
        Compare the words in the transcript with teh keywords set.
    
        Returns a dictionary of five key intentions as boolean values:
        "connect", "clearall","submit", "agree", "disagree"
        Defalut value -  all False
        """
    
        keywords_connect = ['connect','kinect','go','from','build','bridge','add','another','walk','building','going','put','route','train','bridges']
        keywords_clearall = ['clear','delete','remove','clean','erase','empty','cancel','disconnect']
        keywords_submit = ['submit','done','end','finish','terminate']
        keywords_agree = ['yes','yea','okay','agree','ya','like','do','good','great','okay','ok','fine','sure','nevermind']
        keywords_disagree = ['no','not',"don",'disagree','stupid','waste','wasting']
    
        # set up the response object
        intention_dic = {
            "connect": False,
            "clearall": False,
            "submit": False,
            "agree": False,
            "disagree": False
        }
    
        wordList = re.sub("[^\w]"," ", transcript).split()
        print(wordList)
        # set the corresponding intention to true if detected
        if any([keyw in wordList for keyw in keywords_clearall]):
            intention_dic["clearall"] = True
        elif any([keyw in wordList for keyw in keywords_submit]):
            intention_dic["submit"] = True
        elif any([keyw in wordList for keyw in keywords_disagree]):
            intention_dic["disagree"] = True
        elif any([keyw in wordList for keyw in keywords_agree]):
            intention_dic["agree"] = True
        elif any([keyw in wordList for keyw in keywords_connect]):
            intention_dic["connect"] = True 
    
        return intention_dic


def main(args=None):
    rclpy.init(args=args)
    intention_subscriber = IntentionSubscriber()
    # rclpy.spin(intention_subscriber)

    # pyglet.app.run()
    
    # Enter the main event loop.
    while True:
        pyglet.clock.tick()

        for window in pyglet.app.windows:
            window.switch_to()
            window.dispatch_events()
            window.dispatch_event('on_draw')
            window.flip()

        rclpy.spin_once(intention_subscriber, timeout_sec=0.0)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    intention_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
