import React, { useState, useEffect } from "react";
import { Accordion, AccordionSummary, AccordionDetails, TextField, createStyles, makeStyles, Theme, List, Button } from '@material-ui/core';
import ExpandMoreIcon from '@material-ui/icons/ExpandMore';
import { Exhibit } from '../pages/Editor';

const useStyles = makeStyles((theme) =>
    createStyles({
        root: {
            '& .MuiTextField-root': {
                margin: theme.spacing(1)
            },
        },
    }),
);

export const ExhibitSidebarItem = (exhibit) => {
    const classes = useStyles();


    const [exhibitInfo, setExhibitInfo] = useState("");

    return (
        <Accordion square={false}>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
                <div className="exhibit-item-summary">
                    <h2>{exhibit.title}</h2>
                    <p>{exhibit.subtitle}</p>
                </div>
            </AccordionSummary>
            <AccordionDetails>
                <div className="exhibit-item-details">
                    <form className={classes.root}>
                        <TextField required label="Title" defaultValue={exhibit.title} fullWidth />
                        <TextField label="Subtitle" defaultValue={exhibit.subtitle} fullWidth/>
                        <TextField label="Description" multiline defaultValue={exhibit.description} fullWidth/>
                        <TextField label="Start Date" type="date" defaultValue={exhibit.start_date} InputLabelProps={{ shrink: true }} fullWidth/>
                        <TextField label="End Date" type="date" defaultValue={exhibit.end_date} InputLabelProps={{ shrink: true }} fullWidth/>
                        <TextField label="Theme" multiline defaultValue={exhibit.theme} fullWidth />
                        {exhibit.questions.map(question => {
                            return (
                                <TextField label="Question" defaultValue={question} fullWidth />
                            );
                        })}

                        {exhibit.answers.map(answer => {
                            return (
                                <TextField label="Answer" defaultValue={answer} fullWidth />
                            );
                        })}

                        <Button type="submit" variant="contained" >Save</Button>
                    </form>
                </div>
            </AccordionDetails>
        </Accordion>
    );
}