import rclpy
import pyglet
from rclpy.node import Node

from std_msgs.msg import String
# from justhink_world import create_world, load_log, show_world
# from justhink_world.agent import Human, Robot
# from justhink_world.domain.action import PickAction, ClearAction, \
# AttemptSubmitAction, ContinueAction, SubmitAction, AgreeAction, DisagreeAction
# from justhink_world.visual import WorldWindow

class IntentionSubscriber(Node):

    def __init__(self):
        # initialize the activity
        # Create a world.
        # world = create_world('collaboration-1')
        self.instuction_msg = ""

        super().__init__('intention_subscriber')
        self.subscription = self.create_subscription(
            String,
            'intention',
            self.listener_callback,
            100)
        self.subscription  # prevent unused variable warning


        # # Show world
        # win = WorldWindow(world, state_no=None, screen_index=-1)
        # # Override the updater for next label.
        # def _update_next_label():
        #     pass
        # win._update_next_label = _update_next_label
        # # Change the text e.g. in a callback.
        # win.graphics.next_label.text = self.instruction_msg
        # # Enter the main event loop.
        # pyglet.app.run()


    def listener_callback(self, msg):       
        # Excute the action in the activity
        # if msg.data == agree:
        #     world.act(AgreeAction(agent=Human))
        # elif msg.data == disagree:
        #     world.act(DisagreeAction(agent=Human))
    
        if not msg.data == "error":

            intention_dic = self.keyword_detection(msg.data)
            intention = ""
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
                #world.act(AgreeAction(agent=Human))
                #print("User wants to {} ".format(intention))
            elif intention_dic["disagree"]:
                intention = "disagree"
                #world.act(DisagreeAction(agent=Human))
                #print("User wants to {} ".format(intention))
            else:
                print("Error: intention detection failed.")

            self.get_logger().info('User wants to: "%s"' % intention)
            # return intention


   
    def keyword_detection(self,transcript):
        """Detect user's intention when speech recognition succeeds.
        Compare the words in the transcript with teh keywords set.
    
        Returns a dictionary of five key intentions as boolean values:
        "connect", "clearall","submit", "agree", "disagree"
        Defalut value -  all False
        """
    
        keywords_connect = ['connect','kinect','go','from','build','bridge','add','another','walk','building','going','put','route','train']
        keywords_clearall = ['clear','delete','remove','clean','erase','empty','cancel','disconnect']
        keywords_submit = ['submit','done','end','finish','terminate']
        keywords_agree = ['yes','yea','okay','agree','ya','like','do','good','great','okay','ok','fine','sure']
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
    rclpy.spin(intention_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    intention_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
