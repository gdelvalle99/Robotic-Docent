import './App.css';
import axios from 'axios';
import React from 'react';
import _ from "lodash";
import styled from 'styled-components';

const Button = styled.button`
  background-color: black;
  color: white;
  font-size: 20px;
  padding: 10px 60px;
  border-radius: 5px;
  margin: 10px 0px;
  cursor: pointer;
`;

const QuestionPingButton = function(props){
    const {text, onClick} = props;
    return(
        <div>
            <Button onClick={onClick}>{text}</Button>
        </div>
    );
}
const Row = function(props){
    const {question, answer, onClick, showAnswerFlag} = props;
    return (
      <div>
        <Button onClick={onClick}>{question}</Button>
        {showAnswerFlag && (
            <box>{answer}</box>
        )}
      </div>
    );
  }
  
export default class QAGenerator extends React.Component {
    constructor(props){
      super(props);
      this.state = {
        qbutton: [
            {showQuestionsFlag: false, qButtonText: '"I Have A Question!"'}
        ],
        rows: [
          {question: '"Question 1"', answer: 'Answer 1', showAnswerFlag: false},
          {question: '"Question 2"', answer: 'Answer 2', showAnswerFlag: false},
          {question: '"Question 3"', answer: 'Answer 3', showAnswerFlag: false}
        ],
        oldQA: []  
      };

    }
    componentDidMount() {
      this.refreshQA = setInterval(
        () => this.updateList(),
        1000
      );
    }
    componentWillUnmount() {
      clearInterval(this.refreshQA);
    }

    addRow = (Q, A) => {
      const rows = [...this.state.rows, 
                    {question: Q, answer: A, showAnswerFlag: false}
                   ];
      this.setState({
          rows: rows
      });
    }
    
    clearRows = () => {
      this.setState({
        rows: []
      });
    }

    updateList = async () => {
      var newQA = await axios.get(`http://127.0.0.1:5001/server/qa`).
        catch( newQA => newQA);
      const robot_not_found = newQA instanceof Error;
      if(!robot_not_found && !_.isEqual(newQA.data, this.state.oldQA)){
        var QAs = newQA.data;
        this.clearRows();
        QAs.QAList.forEach(QA => {
          this.addRow(QA.question, QA.answer)
        });
        this.setState({
          oldQA: newQA.data
        });
      }
    }
    
    showAnswer = (idx) => {
      const rows = [...this.state.rows];
      rows[idx].showAnswerFlag = !rows[idx].showAnswerFlag;
      this.setState({
          rows: rows
      });
    }
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
    
    render(){
      return(
        <div>
            {this.state.qbutton.map((button) => {
                return(
                    <QuestionPingButton
                            text={button.qButtonText}
                            onClick={this.showQuestions}
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
        </div>
      );
    }
  }
