import React, { useState, useEffect, useRef } from "react";
import { Card } from '@material-ui/core';
import { CardHeader } from '@material-ui/core'; 
import { CardContent} from '@material-ui/core';
import { TextField } from '@material-ui/core';

export const ExhibitModalQuestionCard = (props) => {
    const [isExpanded, setExpanded] = useState(false);
    const [widthClass, setWidthClass] = useState('');
    const [questionFlag, setQuestionFlag] = useState(false);
    const [question, setQuestion] = useState(props.question);
    const [answer, setAnswer] = useState(props.answer);
    const [index, setIndex] = useState(props.index);

    const width30 = 'exhibit-item-qna-width-30';
    const width100 = 'exhibit-item-qna-width-100';

    const updateQuestionSet = () => {
        props.newCard(question, answer);
    }

    //Handle off click textfield auto-save
    const handleQuestionChange = (event) => {
        setQuestion(event.target.value);
        updateQuestionSet();
    }

    const handleAnswerChange = (event) => {
        setAnswer(event.target.value);
        updateQuestionSet();
    }

    //Handles expanding field info and size
    const handleNewCardExpand = () => {
        setExpanded(true);
        setWidthClass(width100);
    }

    const handleToggleCard = () => {
        if (isExpanded) {
            setExpanded(false);
            setWidthClass(width30);
        }
        else {
            setExpanded(true);
            setWidthClass(width100);
        }
    }
    
    const handleCloseCard = () => {
        setExpanded(false);
        setWidthClass(width30);
    }

    //Event handler for clicking away from card component
    const refCheck = useRef(null);

    useEffect(() => {
        const handleClickOff = (event) => {
            if (refCheck.current && !refCheck.current.contains(event.target)) {
                handleCloseCard();
            }
        }

        document.addEventListener("mousedown", handleClickOff);
        return () => {
            document.removeEventListener("mousedown", handleClickOff);
        };
    }, [refCheck]);

    //Initial set for current question, answer, index, and expand state
    const setInitialData = () => {
        setQuestion(props.question);
        setAnswer(props.answer);
        setIndex(props.index);
        setWidthClass(width30);
    }

    //Initial render of existent question and answer
    useEffect(() => { 
        if (props.question == null) {
            handleNewCardExpand();
        }
        else {
            setInitialData();
        }
    }, []);

    return(
        <div key={"exhibit-item-qna-card" + (index+1)} id={"exhibit-item-qna-card-" + (index+1)} className={"exhibit-item-qna-card " + widthClass} ref={refCheck}>
            <Card>
                <CardHeader 
                    title={"Question Set " + (index+1)} 
                    titleTypographyProps={{variant: 'body1'}}
                    subheader={question} 
                    subheaderTypographyProps={{variant: 'body2'}}
                    onClick={handleToggleCard}
                />
                {isExpanded && (
                        <CardContent>
                            <div>
                                <TextField 
                                    className="exhibit-item-question" 
                                    label="Question" 
                                    variant="outlined" 
                                    multiline 
                                    name="questions" 
                                    onBlur={handleQuestionChange} 
                                    defaultValue={question} 
                                    fullWidth 
                                />
                                <TextField 
                                    className="exhibit-item-answer" 
                                    label="Answer" 
                                    variant="outlined" 
                                    multiline 
                                    name="answers" 
                                    onBlur={handleAnswerChange} 
                                    defaultValue={answer} 
                                    fullWidth 
                                />
                            </div>
                        </CardContent>
                    )
                }
            </Card>
        </div>
    );
}