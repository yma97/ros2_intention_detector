import rclpy
import pyglet
import re
from rclpy.node import Node

from std_msgs.msg import String
from justhink_world import create_world, load_log, show_world, world
from justhink_world.agent import Human, Robot
from justhink_world.domain.action import SuggestPickAction, ClearAction, \
AttemptSubmitAction, ContinueAction, AttemptSubmitAction, AgreeAction, DisagreeAction
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
        self.instruction_msg = "Heyy, tell me what you want to do!"

        # Start intention detection
        self.subscription = self.create_subscription(
            String,
            'intention',
            self.listener_callback,
            10)
        print("start recognition")
        self.subscription  # prevent unused variable warning
        print("recognition finish")
        
        # Override the updater for next label.
        def _update_next_label():
            pass
        self.win._update_next_label = _update_next_label
        # Change the text e.g. in a callback.
        self.win.graphics.next_label.text = self.instruction_msg


    def listener_callback(self, msg):  
        self.get_logger().info('I heard: "%s"' % msg.data)
        #self.instruction_msg = 'I heard: "%s"' % msg.data
        #self.win.graphics.next_label.text = self.instruction_msg
        
        if not msg.data == "error":

            wordList = re.sub("[^\w]"," ", msg.data).split()
            print(wordList)
            intention_dic = self.keyword_detection(wordList)
            intention = ""
            #current_agent = Human  #self.world.agent

            if intention_dic["connect"]:
                intention = "connect"
                print("You want to connect")
                self.connect_location(wordList)
            elif intention_dic["clearall"]:
                intention = "clear all"
                self.instruction_msg = "Okay, we clear all"
                self.win.graphics.next_label.text = self.instruction_msg

                if ClearAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(ClearAction(agent=Human))
                else:
                    self.win.execute_action(ClearAction(agent=Robot))
                    # print("You are a Robot. You can't clear all!")
                    # self.instruction_msg = "You are a Robot. You can't clear all!"
                    # self.win.graphics.next_label.text = self.instruction_msg

            elif intention_dic["submit"]:
                intention = "submit"
                self.instruction_msg = "Okay we submit"
                self.win.graphics.next_label.text = self.instruction_msg   

                if AttemptSubmitAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(AttemptSubmitAction(agent=Human))
                else:
                    self.win.execute_action(AttemptSubmitAction(agent=Robot))
                    # print("You are a Robot. You can't submit!")
                    # self.instruction_msg = "You are a Robot. You can't submit!"
                    # self.win.graphics.next_label.text = self.instruction_msg  

            elif intention_dic["agree"]:
                intention = "agree"
  
                self.instruction_msg = "Thanks for agree.Tell me what you want to do now?"
                self.win.graphics.next_label.text = self.instruction_msg     

                if AgreeAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(AgreeAction(agent=Human))
                else:
                    self.win.execute_action(AgreeAction(agent=Robot))
                

            elif intention_dic["disagree"]:
                intention = "disagree"

                self.instruction_msg = "You disagree. Then tell me what you want to do?"
                self.win.graphics.next_label.text = self.instruction_msg

                if DisagreeAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(DisagreeAction(agent=Human))
                else:
                    self.win.execute_action(DisagreeAction(agent=Robot))

            else:
                print("Error: intention detection failed.")
                self.instruction_msg = "Error: intention detection failed."
                self.win.graphics.next_label.text = self.instruction_msg

            # self.instuction_msg = "User wants to {} ".format(intention)
            # self.get_logger().info('User wants to: "%s"' % intention)
            # return intention


   
    def keyword_detection(self,wordList):
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


    def connect_location(self, wordList):
        """If user's intention is to connect, detect the keywords for locations in user's transcript
        Convert the location into node numbers and excute.
        """
        keywords_location = ['montreux','neuchatel','basel','interlaken','bern','zurich','luzern','zermatt','st.gallen','davos']
        wordList = set(wordList)
        loc_1 = ""
        loc_2 = ""
        for w in wordList:
            if w in keywords_location:
                loc_1 = w
                print("1st Detect: "+ w)
                wordList.remove(w)
                break
        for w in wordList:
            if w in keywords_location:
                loc_2 = w
                print("2nd Detect: "+ w)
                wordList.remove(w)
                break
        
        if loc_1 == "" or loc_2 == "": 
            self.instruction_msg = "Invalid locations"
            self.win.graphics.next_label.text = self.instruction_msg
        else:
            self.instruction_msg = "You connect Mount {} and Mount {}".format(loc_1, loc_2)
            self.win.graphics.next_label.text = self.instruction_msg
            (u,v) = self.world.env.state.network.get_edge_ids((loc_1, loc_2)) 
            print("location 1 number: {}".format(u))
            print("location 2 number: {}".format(v))
            if SuggestPickAction((u, v), agent=Human)  in self.world.agent.all_actions:
                self.win.execute_action(SuggestPickAction((u, v), agent=Human))
            else:
                self.win.execute_action(SuggestPickAction((u, v), agent=Robot))
    

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
