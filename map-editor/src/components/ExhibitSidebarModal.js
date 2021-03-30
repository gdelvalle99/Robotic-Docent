import React, { useState, useEffect } from "react";
import { ExhibitModalQuestionSet } from './ExhibitModalQuestionSet';
import { Card } from '@material-ui/core';
import { CardHeader } from '@material-ui/core'; 
import { Dialog } from '@material-ui/core';
import { DialogActions } from '@material-ui/core';
import { DialogContent } from '@material-ui/core';
import { DialogTitle } from '@material-ui/core'; 
import { Divider } from '@material-ui/core';
import { TextField } from '@material-ui/core';
import { Button } from '@material-ui/core';
import { createStyles, makeStyles } from '@material-ui/core/styles';

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
        card: {
            margin: 10,
            padding: 5,
            "&:hover": {
                backgroundColor: '#EBEBEB',
                transition: 'background-color 0.4s ease',
                cursor: 'pointer'
            }
        },
        notchedOutline: {
            borderColor: 'white',
        },
        textFieldColor: {
            "&:focus": {
                backgroundColor: '#EBEBEB',
            }
        },
        cardContainer: {
            margin: 5,
        },
        buttonQuestionStyle: {
            margin: 5,
            border: '2px dashed #EBEBEB'
        }
    }),
);

export const ExhibitSidebarModal = (props) => {
    const classes = useStyles();
    const [exhibit, setExhibit] = useState(props.exhibit);
    const [openModal, setOpenModal] = useState(false);
    const [titleFlag, setTitleFlag] = useState(false);
    const [newFlag, setNewFlag] = useState(true);

    //Handles opening and closing of modal with the validation for the title field being required
    const handleCloseModal = () => {
        if (exhibit.title != null && exhibit.title != "") {
            if (newFlag) {
                const idPromise = props.handleSaveNewExhibit(exhibit);
                idPromise.then(d => {
                    setExhibit({...exhibit, id: d});
                });
                setNewFlag(false);
            }
            else {
                props.handleSaveExistingExhibit(exhibit);
            }
            setTitleFlag(false);
            setOpenModal(false);
        }
        else {
            setTitleFlag(true);
        }
    }

    const handleOpenModal = () => {
        setOpenModal(true);
    }

    //Handles updates for each field
    const handleChange = (event) => {
        handleAllChanges(exhibit, {
            [event.target.name]: event.target.value
        });
    }
    
    const handleAllChanges = (exhibit, diff) => {
        setExhibit({...exhibit, ...diff});
    }

    //Function to update question and answer arrays
    const updateQuestionSet = (questionSet, answerSet) => {
        setExhibit({...exhibit, questions: questionSet, answers: answerSet});
    }

    //Initial set for current exhibits
    const setInitialExhibit = () => {
        setExhibit(props.exhibit);        
    }

    //Initial render of existent exhibits and opens modal on instancing new exhibit
    useEffect(() => {
        if (props.exhibit.title == null) {
            handleOpenModal();
            setNewFlag(true);
        }
        else {
            setNewFlag(false);
        }
        setInitialExhibit(); 
    }, []);

    return (
        <div id={"exhibit-item-id-" + exhibit.id}>
            <Card className={classes.card} onClick={handleOpenModal}>
                <CardHeader 
                    title={exhibit.title}
                    titleTypographyProps={{variant: 'h5'}}
                    subheader={exhibit.subtitle} 
                    subheaderTypographyProps={{variant: 'body1'}}
                />
            </Card>
            <Dialog open={openModal} onClose={handleCloseModal} fullWidth={true}>
                <DialogTitle>
                    <div className="exhibit-item-header">
                        <TextField
                            error={titleFlag}
                            required={true} 
                            className="exhibit-item-title" 
                            InputProps={{
                                classes: {
                                    notchedOutline: classes.notchedOutline,
                                    input: classes.textFieldColor
                                }
                            }}
                            placeholder="Title" 
                            variant="outlined" 
                            name="title" 
                            onBlur={handleChange} 
                            defaultValue={exhibit.title}
                            fullWidth
                        />
                        <TextField 
                            className="exhibit-item-subtitle"
                            InputProps={{
                                classes: {
                                    notchedOutline: classes.notchedOutline,
                                    input: classes.textFieldColor
                                }
                            }}
                            placeholder="Subtitle" 
                            size={"small"}
                            variant="outlined" 
                            name="subtitle" 
                            onBlur={handleChange} 
                            defaultValue={exhibit.subtitle}
                            fullWidth
                        />
                        <Divider variant="middle" />
                    </div>
                </DialogTitle>
                <DialogContent>
                    <div className="exhibit-item-content-container">
                        <TextField 
                            className="exhibit-item-description"
                            InputProps={{
                                classes: {
                                    notchedOutline: classes.notchedOutline,
                                    focused: classes.textFieldColor
                                }
                            }}  
                            label="Description" 
                            variant="outlined" 
                            multiline 
                            rows={5}
                            name="description" 
                            onBlur={handleChange} 
                            defaultValue={exhibit.description} 
                            fullWidth
                        />
                        <div className="exhibit-item-dates">
                            <TextField 
                                className="exhibit-item-startdate" 
                                InputProps={{
                                    classes: {
                                        notchedOutline: classes.notchedOutline,
                                        input: classes.textFieldColor
                                    }
                                }}  
                                label="Start Date" 
                                variant="outlined" 
                                type="date" 
                                name="start_date" 
                                onBlur={handleChange} 
                                defaultValue={convertSimpleISODate(exhibit.start_date)} 
                                InputLabelProps={{ shrink: true }}
                                fullWidth 
                            />
                            <TextField 
                                className="exhibit-item-enddate" 
                                InputProps={{
                                    classes: {
                                        notchedOutline: classes.notchedOutline,
                                        input: classes.textFieldColor
                                    }
                                }}  
                                label="End Date" 
                                variant="outlined" 
                                type="date" 
                                name="end_date" 
                                onBlur={handleChange} 
                                defaultValue={convertSimpleISODate(exhibit.end_date)} 
                                InputLabelProps={{ shrink: true }} 
                                fullWidth
                            />
                        </div>
                        <TextField 
                            className="exhibit-item-theme" 
                            InputProps={{
                                classes: {
                                    notchedOutline: classes.notchedOutline,
                                    input: classes.textFieldColor
                                }
                            }}  
                            label="Theme" 
                            variant="outlined"
                            name="theme" 
                            onBlur={handleChange} 
                            defaultValue={exhibit.theme} 
                            fullWidth 
                        />
                        <div className="exhibit-item-qna-cards">
                            <ExhibitModalQuestionSet questions={exhibit.questions} answers={exhibit.answers} handleSetUpdate={updateQuestionSet} styleContainer={classes.cardContainer} buttonStyle={classes.buttonQuestionStyle}/>
                        </div>
                        <div className="exhibit-delete-button">
                            <Button onClick={() => props.handleDeleteExhibit(exhibit.id)} variant="outlined">Delete</Button>
                        </div>
                    </div>
                </DialogContent>
                <DialogActions>
                    
                </DialogActions>
            </Dialog>
        </div>
    );
}