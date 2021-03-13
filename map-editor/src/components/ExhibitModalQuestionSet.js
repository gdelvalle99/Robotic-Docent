import React, { useState, useEffect } from "react";
import { Grid } from '@material-ui/core';
import { Button } from '@material-ui/core';
import { ExhibitModalQuestionCard } from "./ExhibitModalQuestionCard";
import AddCircleOutlineIcon from '@material-ui/icons/AddCircleOutline';

export const ExhibitModalQuestionSet = (props) => {
    const [questionSet, setQuestionSet] = useState([]);

    const handleAddClick = () => {
        addQuestionCard();
        console.log(questionSet);
    }

    const addQuestionCard = () => {
        setQuestionSet(questionSet => [...questionSet, <ExhibitModalQuestionCard/>]);
    }

    const updateSidebarItem = () => {
        const questions = [];
        const answers = [];
        for (let i=0; i<questionSet.length; i++) {
            questions[i] = questionSet[i].question;
            answers[i] = questionSet[i].answer;
        }
        props.handleSetUpdate(questions, answers);
    }

    const newCard = (question, answer) => {
        const newSet = {
            question: '', 
            answer: ''
        }
        newSet.question = question;
        newSet.answer = answer;
        setQuestionSet(questionSet => [...questionSet, newSet]);
    }

    const setInitialQuestionsAnswers = () => {
        const fullSet = [];
        if (props.questions) {
            const setObject = {
                question: '',
                answer: ''
            }
            for (let i=0; i<props.questions.length; i++) {
                setObject.question = props.questions[i];
                setObject.answer = props.answers[i];
                const setCopy = Object.assign({}, setObject);
                fullSet.push(setCopy);
            }
        }
        setQuestionSet(fullSet);
    }

    useEffect(() => { 
        setInitialQuestionsAnswers();
    }, []);

    return (
        <div className="exhibit-item-qna-container">
            <div className="exhibit-item-qna-header">
                <h4>Questions and Answers</h4>
            </div>
            <div className="exhibit-item-qna-cards-container">
                <Grid container>
                    {questionSet.length > 0 && (
                            questionSet.map((set, index) => {
                                return (
                                    <ExhibitModalQuestionCard key={index} question={set.question} answer={set.answer} updatePSidebarItem={updateSidebarItem} handleNewCardUpdate={newCard} index={index} styleContainer={props.styleContainer}/>
                                );
                            })
                        )
                    }
                    <Button className={props.buttonStyle} onClick={handleAddClick}>
                        <AddCircleOutlineIcon fontSize='large' />
                    </Button>
                </Grid>
            </div>
        </div>
    );
}