import { useState } from "react";
import Chart from "react-google-charts";
import { CircularProgress } from "@material-ui/core";

import { makeStyles } from "@material-ui/core/styles";
import MenuItem from "@material-ui/core/MenuItem";
import FormHelperText from "@material-ui/core/FormHelperText";
import FormControl from "@material-ui/core/FormControl";
import Select from "@material-ui/core/Select";
import OutlinedCard from "./OutlinedCard";
import Set1 from "./Set1";
import Set2 from "./Set2";
import Set3 from "./Set3";

const useStyles = makeStyles((theme) => ({
    formControl: {
        margin: theme.spacing(1),
        minWidth: 300,
        width: "50%",
    },
    selectEmpty: {
        marginTop: theme.spacing(2),
        fontSize: "1em",
        // width: "70%",
    },
}));

export default function FormDisplay({ display, handleChange }) {
    const classes = useStyles();

    return (
        <div>
            <FormControl className={classes.formControl}>
                <Select
                    value={display}
                    onChange={handleChange}
                    displayEmpty
                    className={classes.selectEmpty}
                    // inputProps={{ "aria-label": "Without label" }}
                >
                    <MenuItem value={"Exhibit Information"}>
                        Exhibit Infomation
                    </MenuItem>
                    <MenuItem value={"Daily Information"}>
                        Daily Infomation
                    </MenuItem>
                    <MenuItem value={"Robot Information"}>
                        Robot Infomation
                    </MenuItem>
                </Select>
            </FormControl>
            {display === "Exhibit Information" && <Set1/>}
            {display === "Robot Information" && <Set2 />}
            {display === "Daily Information" && <Set3 />}
        </div>
    );
}
