import React, { useState, useEffect } from "react";
import { Fab } from "@material-ui/core";
import { Button } from "@material-ui/core";
import { Dialog } from "@material-ui/core";
import { TextField } from "@material-ui/core";
import { DialogActions } from "@material-ui/core";
import { DialogContent } from "@material-ui/core";
import { DialogTitle } from "@material-ui/core";
import { Divider } from '@material-ui/core';
import { Select } from "@material-ui/core";
import { MenuItem } from "@material-ui/core";
import { InputLabel } from "@material-ui/core";
import { mapLink, floor_id } from "../links";
import Add from "@material-ui/icons/Add";
// import RoomIcon from '@material-ui/icons/Room';
import axios from "axios";
// import { Room } from "@material-ui/icons";
import SvgIcon from "@material-ui/core/SvgIcon";
// import classes from "*.module.css";
import { createStyles, makeStyles } from '@material-ui/core/styles';


//Set the styles
const useStyles = makeStyles(() => ({
    paper: { minWidth: "500px" },
    notchedOutline: {
        borderColor: 'white',
    },
    textFieldColor: {
        "&:focus": {
            backgroundColor: '#EBEBEB',
        }
    },
}));

export const PieceModal = ({ open, handleClose, piece }) => {
    const classes = useStyles();
    const [titleFlag, setTitleFlag] = useState(false);
    if(!piece) return null


    const handleChange = (event) => {
        // handleAllChanges(piece, {
        //     [event.target.name]: event.target.value
        // });
        setTitleFlag(false);
    }

    return (
        <Dialog
            open={open}
            onClose={handleClose}
            aria-labelledby="form-dialog-title"
            classes={{paper: classes.paper}}
            maxWidth="md"
        >

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
                            defaultValue={piece.title}
                            fullWidth
                            multiline
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
                            defaultValue={piece.author}
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
                            defaultValue={piece.description} 
                            fullWidth
                        />
                </div>
            </DialogContent>
            <DialogActions>
                <Button onClick={handleClose} color="primary">
                    Cancel
                </Button>
                <Button onClick={handleClose} color="primary">
                    Add
                </Button>
            </DialogActions>
        </Dialog>
    );
};

export default PieceModal;
