import React, { useState, useEffect } from "react";
import { Accordion } from '@material-ui/core';
import { Typography } from '@material-ui/core';
import { AccordionSummary } from '@material-ui/core';
import { AccordionDetails }from '@material-ui/core';
import { TextField } from '@material-ui/core';
import { createStyles, makeStyles } from '@material-ui/core/styles';
import { Button } from '@material-ui/core';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';

const convertSimpleISODate = function(dateString) {
    if (dateString) {
        let d = new Date(dateString);

        return d.toISOString().substring(0,10);
    }
    else {
        return null;
    }
}

const useStyles = makeStyles((theme) =>
    createStyles({
        root: {
            '& .MuiTextField-root': {
                margin: theme.spacing(1)
            },
            '& .MuiAccordion-root': {
                margin: theme.spacing(1)
            }
        }
    }),
);

export const ExhibitSidebarItem = (props) => {
    const classes = useStyles();
    const [exhibit, setExhibit] = useState(props);

    const setInitialExhibit = () => {
        setExhibit(props);
    }

    const handleChange = (event) => {
        handleAllChanges(exhibit, {
            [event.target.name]: event.target.value
        });
    }
    
    const handleAllChanges = (exhibit, diff) => {
        setExhibit({...exhibit, ...diff});
    }

    useEffect(() => { setInitialExhibit(); }, []);

    return (
        <Accordion id={"exhibit-item-id-" + exhibit.id} square={false}>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                <div className="exhibit-item-summary">
                    <h2>{exhibit.title}</h2>
                    <p>{exhibit.subtitle}</p>
                </div>
            </AccordionSummary>
            <AccordionDetails>
                <div className="exhibit-item-details">
                    <form className={classes.root}>
                        <TextField required className="exhibit-item-title" label="Title" name="title" onBlur={handleChange} defaultValue={exhibit.title} fullWidth />
                        <TextField className="exhibit-item-subtitle" label="Subtitle" name="subtitle" onBlur={handleChange} defaultValue={exhibit.subtitle} fullWidth/>
                        <TextField className="exhibit-item-description" label="Description" multiline name="description" onBlur={handleChange} defaultValue={exhibit.description} fullWidth/>
                        <TextField className="exhibit-item-startdate" label="Start Date" type="date" name="start_date" onBlur={handleChange} defaultValue={convertSimpleISODate(exhibit.start_date)} InputLabelProps={{ shrink: true }} fullWidth/>
                        <TextField className="exhibit-item-enddate" label="End Date" type="date" name="end_date" onBlur={handleChange} defaultValue={convertSimpleISODate(exhibit.end_date)} InputLabelProps={{ shrink: true }} fullWidth/>
                        <TextField className="exhibit-item-theme" label="Theme" multiline name="theme" onBlur={handleChange} defaultValue={exhibit.theme} fullWidth />
                        <Accordion square={false} className={classes.expanded}>
                            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                                <div className="exhibit-item-question-set">
                                    Set of Questions
                                </div>
                            </AccordionSummary>
                            <AccordionDetails>
                                <div className="exhibit-item-qna-card-container">
                                    {exhibit.questions && exhibit.questions.length > 0
                                        ?   exhibit.questions.map((question, index) => {
                                                const answer = exhibit.answers[index];

                                                return (
                                                    <div key={"exhibit-item-qna-card" + (index+1)} className={"exhibit-item-qna-card" + (index+1)}>
                                                        <p className="exhibit-item-qna-card-title">Questions and Answers {index+1}</p>
                                                        <TextField className="exhibit-item-question" label="Question" multiline name="questions" onBlur={handleChange} defaultValue={question} fullWidth />
                                                        <TextField className="exhibit-item-answer" label="Answer" multiline name="answers" onBlur={handleChange} defaultValue={answer} fullWidth />
                                                    </div>
                                                );
                                            })
                                        :   <p>Temp Add Button</p>
                                    }
                                    <Button type="submit" variant="contained">Add New Question</Button>
                                </div>
                            </AccordionDetails>
                        </Accordion>
                    </form>
                </div>
            </AccordionDetails>
        </Accordion>
    );
}