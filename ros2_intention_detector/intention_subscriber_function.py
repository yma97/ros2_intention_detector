from justhink_world import agent
import rclpy
import pyglet
import re
from rclpy.node import Node

from std_msgs.msg import String
from justhink_world import create_world, load_log, show_world, world
from justhink_world.agent import Human, Robot
from justhink_world.domain.action import SubmitAction, SuggestPickAction, ClearAction, \
AttemptSubmitAction, ContinueAction, AttemptSubmitAction, AgreeAction, DisagreeAction
from justhink_world.visual import WorldWindow

class IntentionSubscriber(Node):

    def __init__(self):
        # Call back to detect user's intention
        super().__init__('intention_subscriber')

        # Create a world & initialization global values
        self.world = create_world('collaboration-1')
        print("initialized world")
        self.win = WorldWindow(self.world, state_no=None, screen_index=-1)
        print("world window intitialized 2")
        self.instruction_msg = "Heyy, tell me what you want to do!"
        self.followup = False
        self.intention = ""
        self.detected_loc = ""
        self.possible_loc = []

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
        
        wordList = re.sub("[^\w]"," ", msg.data).split()
        print(wordList)

        if not msg.data == "error" and not self.followup:
            self.action_detection(wordList)
        
        elif not msg.data == "error" and self.followup:
            self.followup_detection(wordList)


    def action_detection(self, wordList):
            
            intention_dic = self.keyword_detection(wordList)

            if intention_dic["connect"]:
                self.intention = "connect"
                print("You want to connect")
                self.connect_location(wordList)
            
            elif intention_dic["clearall"]:
                self.intention = "clear all"
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
                self.intention = "submit"
                self.instruction_msg = "Are you sure that you want to submit?"
                self.win.graphics.next_label.text = self.instruction_msg   

                if AttemptSubmitAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(AttemptSubmitAction(agent=Human))
                else:
                    self.win.execute_action(AttemptSubmitAction(agent=Robot))
                    # print("You are a Robot. You can't submit!")
                    # self.instruction_msg = "You are a Robot. You can't submit!"
                    # self.win.graphics.next_label.text = self.instruction_msg  
                self.followup = True

            elif intention_dic["agree"]:
                self.intention = "agree"
  
                self.instruction_msg = "Thanks for agree.Tell me what you want to do now?"
                self.win.graphics.next_label.text = self.instruction_msg     

                if AgreeAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(AgreeAction(agent=Human))
                else:
                    self.win.execute_action(AgreeAction(agent=Robot))
                

            elif intention_dic["disagree"]:
                self.intention = "disagree"

                self.instruction_msg = "You disagree. Then tell me what you want to do?"
                self.win.graphics.next_label.text = self.instruction_msg

                if DisagreeAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(DisagreeAction(agent=Human))
                else:
                    self.win.execute_action(DisagreeAction(agent=Robot))
            
            elif intention_dic["stupid"]:
                self.intention = "disagree"

                self.instruction_msg = "Thank you. You too ^^"
                self.win.graphics.next_label.text = self.instruction_msg

                if DisagreeAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(DisagreeAction(agent=Human))
                else:
                    self.win.execute_action(DisagreeAction(agent=Robot))                      

            else:
                print("Error: intention detection failed.")
                self.instruction_msg = "Error: intention detection failed: {}".format(wordList)
                self.win.graphics.next_label.text = self.instruction_msg

     
    
    def followup_detection(self, wordList):
        
        intention_dic = self.keyword_detection(wordList)

        if self.intention == "submit":
            if intention_dic["agree"]:
                self.instruction_msg = "Okay we submit!"
                self.win.graphics.next_label.text = self.instruction_msg     

                if SubmitAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(SubmitAction(agent=Human))
                elif SubmitAction(agent=Robot) in self.world.agent.all_actions:
                    self.win.execute_action(SubmitAction(agent=Robot))
                else:
                    print("Error: Submition failed")
                    self.instruction_msg = "Error: Submition failed"
                    self.win.graphics.next_label.text = self.instruction_msg  
            elif intention_dic["disagree"] or intention_dic['clearall']:
                self.instruction_msg = "You canceled your submition!"
                self.win.graphics.next_label.text = self.instruction_msg     

                if ContinueAction(agent=Human) in self.world.agent.all_actions:
                    self.win.execute_action(ContinueAction(agent=Human))
                elif ContinueAction(agent=Robot) in self.world.agent.all_actions:
                    self.win.execute_action(ContinueAction(agent=Robot))
                else:
                    print("Error: Cancellation failed")
                    self.instruction_msg = "Error: Cancellation failed"
                    self.win.graphics.next_label.text = self.instruction_msg  
            self.followup = False

        # possible location list is not empty
        elif self.possible_loc:
            if intention_dic["agree"]:
                loc_2 = self.possible_loc[0]
                self.execute_connection(self.detected_loc, loc_2)
                self.detected_loc = ""
                self.possible_loc.clear()
                self.followup = False
            elif intention_dic["disagree"]:
                self.possible_loc.remove(self.possible_loc[0])
                self.instruction_msg = "Do you want to connect {} and {}?".format(self.detected_loc, self.possible_loc[0])
                self.win.graphics.next_label.text = self.instruction_msg
            else:
                print("Error: Invalid command. Please say yes or no!")
                self.instruction_msg = "Error: Invalid command. Please say yes or no!"
                self.win.graphics.next_label.text = self.instruction_msg  

    def keyword_detection(self,wordList):
        """Detect user's intention when speech recognition succeeds.
        Compare the words in the transcript with teh keywords set.
    
        Returns a dictionary of five key intentions as boolean values:
        "connect", "clearall","submit", "agree", "disagree"
        Defalut value -  all False
        """
    
        keywords_connect = ['connect','kinect','go','from','build','bridge','add','another','walk','building','going','put','route','train','bridges']
        keywords_clearall = ['clear','delete','remove','clean','erase','empty','cancel','disconnect']
        keywords_submit = ['submit','done','end','finish','terminate','finished']
        keywords_agree = ['yes','yea','okay','agree','ya','like','do','good','great','okay','ok','fine','sure','nevermind']
        keywords_disagree = ['no','nope','not',"don",'disagree','waste','wasting']
        keywords_badwords = ['stupid']

        # set up the response object
        intention_dic = {
            "connect": False,
            "clearall": False,
            "submit": False,
            "agree": False,
            "disagree": False,
            "stupid": False
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
        elif any([keyw in wordList for keyw in keywords_badwords]):
            intention_dic["stupid"] = True

        return intention_dic


    def connect_location(self, wordList):
        """If user's intention is to connect, detect the keywords for locations in user's transcript
        Convert the location into node numbers and excute.
        """
        keywords_location = ['montreux','neuchatel','basel','interlaken','bern','zurich','luzern', 'lucerne','zermatt','st.gallen','davos']
        wordList = set(wordList)
        loc_1 = ""
        loc_2 = ""
        for w in wordList:
            if w in keywords_location:
                loc_1 = w
                if w == "lucerne": loc_1 = "luzern"
                print("1st Detect: "+ w)
                wordList.remove(w)
                break
        for w in wordList:
            if w in keywords_location:
                loc_2 = w
                if w == "lucerne": loc_1 = "luzern"
                print("2nd Detect: "+ w)
                wordList.remove(w)
                break
        
        # If both of the location are invalid, re-ask for the full input
        if loc_1 == "" and loc_2 == "": 
            self.instruction_msg = "Invalid locations: {}".format(wordList)
            self.win.graphics.next_label.text = self.instruction_msg
        
        # If one of the location are valid, ask for the other input
        elif not loc_1 == "" and loc_2 == "": 
            self.followup = True
            self.detected_loc = loc_1
            
            # Get the action list with the names.
            action_list = []
            for action in sorted(self.world.agent.all_actions):
                if hasattr(action, 'edge'):
                    u, v = self.world.env.state.network.get_edge_name(action.edge)
                    action = action.__class__(edge=(u, v), agent=action.agent)
                    action_list.append(action)
            # Filtering for actions from detected location.
            for action in action_list:
                if isinstance(action, SuggestPickAction) and action.edge[0] == loc_1.capitalize():
                    self.possible_loc.append(action.edge[1])

            print(self.possible_loc)
            ##############################
            # need to modify - add edge case - if no available action
            ##############################
            self.instruction_msg = "Do you want to connect {} and {}?".format(loc_1, self.possible_loc[0])
            self.win.graphics.next_label.text = self.instruction_msg
        
        # Get both valid locations, excute the detection
        else:
            self.execute_connection(loc_1, loc_2)
    
    def execute_connection(self, u, v):
            self.instruction_msg = "You connect Mount {} and Mount {}".format(u, v)
            self.win.graphics.next_label.text = self.instruction_msg
            (u,v) = self.world.env.state.network.get_edge_ids((u, v)) 
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
