import './App.css';
import axios from 'axios';
import React from 'react';
import _ from "lodash";
import styled from 'styled-components';

const robot_url = "http://d5528505995f.ngrok.io";

//styles the button ui elements to look nice
const Button = styled.button` 
  background-color: #55AD74;
  color: white;
  font-size: 20px;
  padding: 10px 90px;
  border-radius: 5px;
  margin: 10px 0px;
  cursor: pointer;
`;

const MButton = styled.button` 
  background-color: royalblue;
  color: white;
  font-size: 20px;
  padding: 10px 20px;
  border-radius: 5px;
  margin: 10px 0px;
  cursor: pointer;
`;

const QButton = styled.button` 
  background-color: #1E5D88;
  color: white;
  font-size: 20px;
  width: 240px;
  min-height: 50px;
  border-radius: 5px;
  margin: 10px 0px;
  padding: 15px 15px;
  cursor: pointer;
`;

//Button that controls the dispay of the current questions/answer for the tour
const QuestionPingButton = function(props){
    const {text, onClick, displayFlag} = props;
    return(
        <div>
          {displayFlag &&
            <Button onClick={onClick}>{text}</Button>
          }
        </div>
    );
}

//behavior for question buttons
//on click, textbox pops up with answet text
const Row = function(props){
    const {question, answer, onClick, showAnswerFlag} = props;
    return (
      <div>
        <QButton onClick={onClick}>{question}</QButton>
        <br></br>
        {showAnswerFlag && (
            <box>{answer}</box>
        )}
      </div>
    );
  }

const MRow = function(props){
    const {question, answer, onClick, showAnswerFlag} = props;
    return (
      <div>
        <MButton onClick={onClick}>{question}</MButton>
        <br></br>
        {showAnswerFlag && (
            <mbox>{answer}</mbox>
        )}
      </div>
    );
  }
  
export default class QAGenerator extends React.Component {
    constructor(props){
      super(props);
      this.state = {
        qbutton: [
            {showQuestionsFlag: false, qButtonText: '"I Have A Question!"', displayFlag: true}
        ],
        mapbutton: [
          {question: 'Map', answer: null, showAnswerFlag: false}
        ],
        rows: [//default (bunk) questions
          {question: '"What material is this made out of?"', answer: 'Copper, steel, and various other metals', showAnswerFlag: false},
          {question: '"Where did the artist make this piece?"', answer: 'Tehran, Iran', showAnswerFlag: false},
          {question: '"What are the main themes of the piece?"', answer: 'Love, peace, and war', showAnswerFlag: false}
        ],
        oldQA: [], //flag for updating question boxes 
        oldMap: null
      };

    }
    componentDidMount() {
      this.refreshQA = setInterval( //pings robot every second
        () => this.updateList(),
        5000
      );
    }
    componentWillUnmount() {
      clearInterval(this.refreshQA); //stops pinging robot on death
    }

    //behavior for adding new question and answer boxes
    addRow = (Q, A) => {
      const rows = [...this.state.rows, 
                    {question: Q, answer: A, showAnswerFlag: false}
                   ];
      this.setState({
          rows: rows
      });
    }
    
    //behavior to delete all QA boxes when updating list
    clearRows = () => {
      this.setState({
        rows: []
      });
    }

    updateList = async () => {
      var newQA = await axios.get(robot_url + `/server/qa`). //robot server
        catch( newQA => newQA);
      const robot_not_found = newQA instanceof Error; //error only occurs when robot server not found
      if(!robot_not_found && !_.isEqual(newQA.data, this.state.oldQA)){//checks that new QA data was pulled from robot server
        var QAs = newQA.data;
        this.clearRows(); //clear old data
        QAs.QAList.forEach(QA => {
          this.addRow(QA.question, QA.answer) //populate screen with new QA
        });
        this.setState({
          oldQA: newQA.data
        });
      }
    }
    
    //behavior for clicking on Question box
    //should populatescreen with associated asnwer
    //hides answer on second click
    showAnswer = (idx) => {
      const rows = [...this.state.rows];
      rows[idx].showAnswerFlag = !rows[idx].showAnswerFlag;
      const data = {Q: rows[idx].question, A: rows[idx].answer};
      if(rows[idx].showAnswerFlag){
        const text2speech = axios.post(robot_url + `/send_answer`, data);
      }
      
      this.setState({
          rows: rows
      });
    }
    //behavior of the "I Have A Question" button
    //on click, populates screen with questions and change text to "Thank You!"
    //clicking "Thank You!" hides QA boxes and changes text back
    showQuestions = () => {
        const qbutton = [...this.state.qbutton];
        const rows = [...this.state.rows];
        qbutton[0].showQuestionsFlag = !qbutton[0].showQuestionsFlag;
        if(!qbutton[0].showQuestionsFlag){
            qbutton[0].qButtonText = '"I Have A Question!"';
            rows.forEach(row => {
                row.showAnswerFlag = false
            });
        }
        else{
            qbutton[0].qButtonText = '"Thank You!"';
        } 
        this.setState({
            qbutton: qbutton
        });
    }



    displayMap = async () => {
      const mapbutton = [...this.state.mapbutton];
      /*var museum_map = await axios.get(`http://127.0.0.1:5001/server/qa`)
        catch( museum_map => museum_map);*/
      const robot_not_found = false//museum_map instanceof Error; //error only occurs when robot server not found
      if(robot_not_found){
        mapbutton[0].answer = React.createElement(
          "img",
          {
            src: process.env.PUBLIC_URL + '/map_not_found',
            width: "303px",
            height: "220px"
          });
      }
      else{
        mapbutton[0].answer = React.createElement(
          "img",
          {
            src: process.env.PUBLIC_URL + '/map.png',
            width: "303px",
            height: "220px"
          });
      }
      mapbutton[0].showAnswerFlag = !mapbutton[0].showAnswerFlag;
      this.setState({
        //oldMap: museum_map,
        mapbutton: mapbutton

      });
    }
    
    //handles real-time rendering of new QA
    //handles proper placement of UI elements
    render(){
      return(
        <div>
          <br></br>
          <br></br>
          <br></br>
          <br></br>
            {this.state.qbutton.map((button) => {
                return(
                    <QuestionPingButton
                            text={button.qButtonText}
                            onClick={this.showQuestions}
                            displayFlag={button.displayFlag}
                    />
                )
                })
            }
          {this.state.rows.map((row, idx) => {
            return(
                <div>
                    {this.state.qbutton[0].showQuestionsFlag && 
                        (<Row 
                        key={idx} 
                        question={row.question}
                        answer={row.answer}
                        onClick={() => this.showAnswer(idx)}
                        showAnswerFlag={row.showAnswerFlag}
                    />)} 
                </div>
              )
          })
          }
          <br></br>
          <br></br>
          <br></br>
          {this.state.mapbutton.map((row, idx) => {
            return(
                <div>
                    {(<MRow 
                        key={idx} 
                        question={row.question}
                        answer={row.answer}
                        onClick={() => this.displayMap(idx)}
                        showAnswerFlag={row.showAnswerFlag}
                    />)} 
                </div>
                )
              })
            }
        </div>
      );
    }
  }
